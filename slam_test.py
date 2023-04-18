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
import utils_ref, sensors, vehicles


def main(args):
    try:
        ### CARLA Setup ###
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        
        world = client.get_world()

        ### Vehicle Setup ###
        vehicle_manager = vehicles.VehicleManager(world)
        vehicle_manager.spawn_vehicle(args.filter, 'autopilot')

        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.ignore_lights_percentage(vehicle_manager.vehicle, 100)
        settings.fixed_delta_seconds = args.delta
        settings.synchronous_mode = True
        world.apply_settings(settings)

        ### Lidar Setup ###
        lidar_manager = sensors.LidarManager(args, world)
        lidar_manager.spawn_lidar(vehicle_manager.vehicle)
        lidar_manager.start_vis()
        
        ### Main Loop ###
        utils_ref.vis_loop(world, lidar_manager)

    except KeyboardInterrupt:
        print('\ndestroying actors')
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle_manager.vehicle.destroy()
        lidar_manager.lidar.destroy()
        lidar_manager.vis.destroy_window()
        print('done.')

if __name__ == '__main__':
    args = utils_ref.parse_carla_args()
    main(args)