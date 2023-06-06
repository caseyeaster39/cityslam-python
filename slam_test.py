# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
import sys

try:
    sys.path.append(glob.glob('%s/carla/dist/carla-*%d.%d-%s.egg' % (
        os.getenv('CARLA_HOME'),
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import gtsam
import time
import argparse
import matplotlib.pyplot as plt

import v2x

# Reference: 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB): open3d_lidar.
def parse_carla_args():
    argparser = argparse.ArgumentParser(
        description="CARLA VSLAM Test")
    argparser.add_argument(
        '-d', '--debug',
        action='store_true',
        help='Print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument(
        '--delta',
        default=0.05,
        type=float,
        help='delta time between frames in seconds (default: 0.05)')
    argparser.add_argument(
        '--no-noise',
        action='store_true',
        help='remove the drop off and noise from the lidar')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='model3',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--upper-fov',
        default=15.0,
        type=float,
        help='lidar\'s upper field of view in degrees (default: 15.0)')
    argparser.add_argument(
        '--lower-fov',
        default=-25.0,
        type=float,
        help='lidar\'s lower field of view in degrees (default: -25.0)')
    argparser.add_argument(
        '--channels',
        default=64.0,
        type=float,
        help='lidar\'s channel count (default: 64)')
    argparser.add_argument(
        '--range',
        default=100.0,
        type=float,
        help='lidar\'s maximum range in meters (default: 100.0)')
    argparser.add_argument(
        '--points-per-second',
        default=500000,
        type=int,
        help='lidar\'s points per second (default: 500000)')
    argparser.add_argument(
        '--keyframe-interval',
        default=20,
        type=int,
        help='lidar\'s number of scans between lidar keyframes (default: 20)')
    argparser.add_argument(
        '--ping-every',
        default=50,
        type=int,
        help='number of carla.tick() calls between v2x pings (default: 50)')
    argparser.add_argument(
        '--rsu-queue-size',
        default=2,
        type=int,
        help='number of carla.tick() calls between v2x pings (default: 2)')
    argparser.add_argument(
        '-x',
        default=0.0,
        type=float,
        help='offset in the sensor position in the X-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-y',
        default=0.0,
        type=float,
        help='offset in the sensor position in the Y-axis in meters (default: 0.0)')
    argparser.add_argument(
        '-z',
        default=0.0,
        type=float,
        help='offset in the sensor position in the Z-axis in meters (default: 0.0)')
    args = argparser.parse_args()
    return args


def visualize_pose_graph(graph, fig_name=None):
    graph_factors = graph[0]
    graph_values = graph[1]

    # Plot the poses in 3D space
    fig = plt.figure()
    if fig_name is not None:
        fig.canvas.manager.set_window_title(fig_name)
    ax = fig.add_subplot(111)
    drawn = {}

    for idx in range(graph_factors.size()):
        factor = graph_factors.at(idx)
        for key in factor.keys():
            if key not in drawn.keys():
                position = graph_values.atPose3(key).translation()
                # orientation = graph_values.atPose3(key).matrix()
                label = f"{chr(gtsam.symbolChr(key))}{gtsam.symbolIndex(key)}"
                ax.scatter(position[0], position[1], marker='o')
                ax.text(position[0], position[1], label)
                drawn[key] = position
        if isinstance(factor, gtsam.BetweenFactorPose3):
            [key1, key2] = factor.keys()
            ax.plot([drawn[key1][0], drawn[key2][0]],
                    [drawn[key1][1], drawn[key2][1]],
                    [drawn[key1][2], drawn[key2][2]], color='k')
    input()


def vis_loop(world, v2x_manager, spectator):
    vehicle = v2x_manager.vehicle_list[0]
    spectator_transform = carla.Transform(carla.Location(x=0, y=0, z=50), carla.Rotation(pitch=-90))
    spectator.set_transform(vehicle.vehicle.get_transform())

    point_cloud = vehicle.brain.get_sensor_data('Lidar')
    pc_vis = vehicle.brain.get_sensor_vis('Lidar')
    
    frame = 0
    while True:
        if frame == 2: # Wait until a frame is available to be added
            pc_vis.add_geometry(point_cloud)
        if frame % v2x_manager.args.ping_every == 0:
            v2x_manager.ping()

        # TODO: Make this less jerky
        spectator_transform.location = vehicle.vehicle.get_location() + carla.Location(z=50)
        spectator_transform.rotation.yaw = vehicle.vehicle.get_transform().rotation.yaw
        spectator.set_transform(spectator_transform)
            
        pc_vis.update_geometry(point_cloud)
        pc_vis.poll_events()
        pc_vis.update_renderer()
        
        time.sleep(0.005) 
        world.tick()
        frame += 1


def main(args, pose_vis=True):
    try:
        ### CARLA Setup ###
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        
        world = client.get_world()
        v2x_manager = v2x.V2X_Manager(world, args)

        ### Vehicle Setup ###
        spawn_location = carla.Transform(carla.Location(x=-103.6, y=85.8, z=0.25), 
                                         carla.Rotation(yaw=270))
        v2x_manager.add_vehicle(args.filter, 'autopilot', spawn_location=spawn_location, sensors=['Lidar'], vis=True)
        vehicle_manager = v2x_manager.vehicle_list[0]

        ### Wold Setup ###
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.ignore_lights_percentage(vehicle_manager.vehicle, 100)
        settings.fixed_delta_seconds = args.delta
        settings.synchronous_mode = True
        world.apply_settings(settings)    

        spectator = world.get_spectator()

        ### RSU Setup ###
        v2x_manager.add_rsu(carla.Location(x=-44.4, y=18.8, z=2.5), 44, [])
        
        ### Main Loop ###
        vis_loop(world, v2x_manager, spectator)

    except KeyboardInterrupt:
        graph = v2x_manager.rsu_list[0].brain.get_graph()
        print('\ndestroying actors')
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle_manager.destroy_actors()
        print('done.')

    if pose_vis:
        plt.ion()
        visualize_pose_graph(graph, 'Pose Graph')
        # for idx in range(40, 60):
        #     v2x_manager.rsu_list[0].brain.memory_manager.loop_detector.plotNode(idx)

if __name__ == '__main__':
    args = parse_carla_args()
    main(args)