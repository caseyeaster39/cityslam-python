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
import utils_ref, v2x


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
        v2x_manager.add_rsu(carla.Transform(carla.Location(x=-44.4, y=18.8, z=2.5)), 50, [])
        
        ### Main Loop ###
        utils_ref.vis_loop(world, vehicle_manager)

    except KeyboardInterrupt:
        print('\ndestroying actors')
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)

        vehicle_manager.destroy_actors()
        print('done.')

if __name__ == '__main__':
    args = utils_ref.parse_carla_args()
    main(args)