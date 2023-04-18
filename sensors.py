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
import open3d as o3d
import numpy as np
from matplotlib import cm

import utils_ref, pose_graph

# Reference: 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB): open3d_lidar.
class LidarManager:
    def __init__(self, args, world) -> None:
        ### Lidar Setup ###
        self.world = world
        self.lidar_bp = self.bp_init(args)
        offset = carla.Location(args.x, args.y, args.z)
        self.lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8) + offset)
        self.lidar = None
        
        ### Open3d Setup ###
        self.pc = o3d.geometry.PointCloud()
        self.vis = None
        self.color_map = np.array(cm.get_cmap('plasma').colors)
        self.color_range = np.linspace(0.0, 1.0, self.color_map.shape[0])

        ### SLAM Setup ###
        self.frame_counter = 1
        self.keyframe_interval = args.keyframe_interval

        self.pose_graph_manager = pose_graph.PoseGraphManager()
        self.pose_graph_manager.add_prior()
        self.pose_graph_manager.set_loop_detector(utils_ref.ScanContextManager())

    def bp_init(self, args):
        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.find('sensor.lidar.ray_cast')
        if args.no_noise:
            bp.set_attribute('dropoff_general_rate', '0.0')
            bp.set_attribute('dropoff_intensity_limit', '1.0')
            bp.set_attribute('dropoff_zero_intensity', '0.0')
        else:
            bp.set_attribute('noise_stddev', '0.2')
        bp.set_attribute('upper_fov', str(args.upper_fov))
        bp.set_attribute('lower_fov', str(args.lower_fov))
        bp.set_attribute('channels', str(args.channels))
        bp.set_attribute('range', str(args.range))
        bp.set_attribute('rotation_frequency', str(1.0 / args.delta))
        bp.set_attribute('points_per_second', str(args.points_per_second))
        return bp
    
    def spawn_lidar(self, vehicle):
        self.lidar = self.world.spawn_actor(self.lidar_bp, 
                                            self.lidar_transform, 
                                            attach_to=vehicle)
        self.lidar.listen(lambda data: self.lidar_callback(data))
    
    def lidar_callback(self, point_cloud):
        coords, int_color = self.disp_lidar(point_cloud)

        self.pc.points = o3d.utility.Vector3dVector(coords)
        self.pc.colors = o3d.utility.Vector3dVector(int_color)

        if not self.frame_counter % self.keyframe_interval: # Every [interval] frames
            self.pose_graph_manager.update(self.pc)
        self.frame_counter += 1

    def disp_lidar(self, point_cloud):
        data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
        data = np.reshape(data, (int(data.shape[0] / 4), 4))

        # Coloring by intensity of each point
        intensity = data[:, -1]
        log_col = 1.0 - np.log(intensity) / -0.4
        int_color = np.c_[
            np.interp(log_col, self.color_range, self.color_map[:, 0]),
            np.interp(log_col, self.color_range, self.color_map[:, 1]),
            np.interp(log_col, self.color_range, self.color_map[:, 2])]

        # Correcting between carla and open3d coordinate systems
        coords = data[:, :-1]
        coords[:, :1] *= -1

        return coords, int_color

    def start_vis(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name='Carla Lidar',
            width=960,
            height=540,
            left=480,
            top=270)
        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True        
        self.vis = vis
