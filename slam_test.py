# References:   2020 Computer Vision Center (CVC) at the Universitat Autonoma de
#               Barcelona (UAB) -> open3d_lidar.py

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
import matplotlib.pyplot as plt

import utils_ref, v2x


def visualize_pose_graph(graph, fig_name=None):
    graph_factors = graph[0]
    graph_values = graph[1]

    # Plot the poses in 3D space
    fig = plt.figure()
    if fig_name is not None:
        fig.canvas.manager.set_window_title(fig_name)
    ax = fig.add_subplot(111, projection='3d')
    drawn = {}

    for idx in range(graph_factors.size()):
        factor = graph_factors.at(idx)
        for key in factor.keys():
            if key not in drawn.keys():
                position = graph_values.atPose3(key).translation()
                # orientation = graph_values.atPose3(key).matrix()
                label = f"{chr(gtsam.symbolChr(key))}{gtsam.symbolIndex(key)}"
                ax.scatter(position[0], position[1], position[2], marker='o')
                ax.text(position[0], position[1], position[2], label)
                drawn[key] = position
        if isinstance(factor, gtsam.BetweenFactorPose3):
            [key1, key2] = factor.keys()
            ax.plot([drawn[key1][0], drawn[key2][0]],
                    [drawn[key1][1], drawn[key2][1]],
                    [drawn[key1][2], drawn[key2][2]], color='k')
    input()


def main(args):
    try:
        ### CARLA Setup ###
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        
        world = client.get_world()
        v2x_manager = v2x.V2X_Manager(world, args)

        ### Vehicle Setup ###
        spawn_location = carla.Transform(carla.Location(x=0, y=16.7, z=0.5), 
                                         carla.Rotation(yaw=180))
        v2x_manager.add_vehicle(args.filter, 'autopilot', location=spawn_location, sensors=['Lidar'], vis=True)
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

        ### RSU Setup ###
        v2x_manager.add_rsu(carla.Location(x=-44.4, y=18.8, z=2.5), 50, [])
        
        ### Main Loop ###
        utils_ref.vis_loop(world, v2x_manager)

    except KeyboardInterrupt:
        gf = vehicle_manager.lidar_manager.pose_graph_manager.graph_factors
        gv = vehicle_manager.lidar_manager.pose_graph_manager.graph_values
        print('\ndestroying actors')
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle_manager.destroy_actors()
        print('done.')

    plt.ion()
    visualize_pose_graph((gf, gv), 'Pose Graph')

if __name__ == '__main__':
    args = utils_ref.parse_carla_args()
    main(args)