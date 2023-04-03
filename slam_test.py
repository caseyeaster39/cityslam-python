# References: 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB) -> open3d_lidar.py

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
import random
import utils_ref, sensors


def main(args):
    try:
        ### CARLA Setup ###
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        settings.fixed_delta_seconds = args.delta
        settings.synchronous_mode = True
        world.apply_settings(settings)

        ### Vehicle Setup ###
        bp = blueprint_library.filter(args.filter)[0]
        ego_init = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, ego_init)
        print('created %s' % vehicle.type_id)

        vehicle.set_autopilot(True)

        ### Lidar Setup ###
        lidar_manager = sensors.LidarManager(args, world)
        lidar_manager.spawn_lidar(vehicle)
        lidar_manager.start_vis('Lidar')
        lidar_manager.start_vis('ICP')
        

        utils_ref.vis_loop(world, lidar_manager)

    except KeyboardInterrupt:
        print('\ndestroying actors')
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle.destroy()
        lidar_manager.lidar.destroy()
        for vis in lidar_manager.vis.values():
            vis.destroy_window()
        print('done.')

if __name__ == '__main__':
    args = utils_ref.parse_carla_args()
    main(args)