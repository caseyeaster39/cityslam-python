import sys
import argparse
import time
from datetime import datetime

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


def vis_loop(world, vis, point_cloud): # From CVC @ UAB
    frame = 0
    dt0 = datetime.now()
    while True:
        if frame == 2:
            vis.add_geometry(point_cloud)
        vis.update_geometry(point_cloud)

        vis.poll_events()
        vis.update_renderer()
        
        time.sleep(0.005) # Fix Open3d jittering -> per reference commment
        world.tick()

        process_time = datetime.now() - dt0
        sys.stdout.write('\r' + 'FPS: ' + str(1.0 / process_time.total_seconds()))
        sys.stdout.flush()
        dt0 = datetime.now()
        frame += 1