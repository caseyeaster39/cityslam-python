import copy
import math
import gtsam
import numpy as np
import open3d as o3d


class PoseGraphManager:
    def __init__(self) -> None:
        self.prior_cov = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        self.constraint_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])
        self.odometry_cov = gtsam.noiseModel.Diagonal.Sigmas(self.constraint_cov)
        self.loop_cov = gtsam.noiseModel.Diagonal.Sigmas(self.constraint_cov)

        self.graph_factors = gtsam.NonlinearFactorGraph()
        self.graph_values = gtsam.Values()

        self.current_pose = np.eye(4)
        self.previous_pc = None

        self.current_node_idx = 0
        self.previous_node_idx = 0

        self.loop_detector = None
        self.icp_init = np.eye(4)
        self.graph_optimized = None

    def set_loop_detector(self, loop_detector):
        self.loop_detector = loop_detector

    def add_prior(self):
        self.graph_factors.add(
            gtsam.PriorFactorPose3(gtsam.symbol('x', self.current_node_idx), 
                                   gtsam.Pose3(self.current_pose), 
                                   self.prior_cov))
        self.graph_values.insert(gtsam.symbol('x', self.current_node_idx), gtsam.Pose3(self.current_pose))
        self.current_node_idx += 1

    def add_odometry(self, odom_transform):
        self.graph_factors.add(
            gtsam.BetweenFactorPose3(gtsam.symbol('x', self.previous_node_idx), 
                                     gtsam.symbol('x', self.current_node_idx), 
                                     gtsam.Pose3(odom_transform), 
                                     self.odometry_cov))
        self.graph_values.insert(gtsam.symbol('x', self.current_node_idx), gtsam.Pose3(self.current_pose))

    def add_loop(self, loop_transform, loop_idx):
        self.graph_factors.add(
            gtsam.BetweenFactorPose3(gtsam.symbol('x', loop_idx), 
                                     gtsam.symbol('x', self.current_node_idx), 
                                     gtsam.Pose3(loop_transform), 
                                     self.loop_cov))
        
    def optimize(self):
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph_factors, 
                                                           self.graph_values, 
                                                           gtsam.LevenbergMarquardtParams())
        self.graph_optimized = optimizer.optimize()

        pose = self.graph_optimized.atPose3(gtsam.symbol('x', self.current_node_idx))

        self.current_pose[:3, 3] = np.array([pose.x(), pose.y(), pose.z()])    # Translation
        self.current_pose[:3, :3] = pose.rotation().matrix()                   # Rotation

    def update(self, point_cloud):
        print(f"\nCurrent node: {self.current_node_idx}")
        if self.current_node_idx == 1:
            self.previous_pc = copy.deepcopy(point_cloud)
        pc_downsampled = point_cloud.uniform_down_sample(every_k_points=10)
        pc_previous_downsampled = self.previous_pc.uniform_down_sample(every_k_points=10)

        self.loop_detector.addNode(node_idx=self.current_node_idx-1,
                                   ptcloud=pc_downsampled)

        odom_transform = self.icp(pc_downsampled, pc_previous_downsampled)

        self.current_pose = np.matmul(self.current_pose, odom_transform)
        self.add_odometry(odom_transform)
        self.icp_init = odom_transform

        if (self.current_node_idx > 0 and self.current_node_idx % 10 == 0):
            loop_idx, _, yaw_diff_deg = self.loop_detector.detectLoop()
            if loop_idx is not None:
                print('Loop detected between node {} and {}'.format(self.current_node_idx, loop_idx))
                pc_loop_downsampled = self.loop_detector.ptclouds[loop_idx]
                loop_transform = self.icp(pc_downsampled,
                                          pc_loop_downsampled,
                                          init_pose=self.yawdeg2se3(yaw_diff_deg))
                self.add_loop(loop_transform, loop_idx)
                self.optimize()

        self.previous_pc = copy.deepcopy(point_cloud)
        self.previous_node_idx = self.current_node_idx
        self.current_node_idx += 1

    def icp(self, pc_downsampled, pc_previous_downsampled, init_pose=None):
        reg_p2p = o3d.pipelines.registration.registration_icp(source = pc_downsampled, 
                                                              target = pc_previous_downsampled, 
                                                              max_correspondence_distance = 10, 
                                                              init = self.icp_init if init_pose is None else init_pose, 
                                                              estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
                                                              criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
        return reg_p2p.transformation
    
    def yawdeg2se3(self, yaw_deg):
        yaw_rad = np.deg2rad(yaw_deg)
        se3 = np.eye(4)
        se3[:3, :3] = self.eulerAnglesToRotationMatrix([0, 0, yaw_rad])
        return se3 

    @staticmethod
    def eulerAnglesToRotationMatrix(yaw) :
        
        # assume roll = 0
        R_x = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

        # assume pitch = 0
        R_y = np.array([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])

        # yaw
        R_z = np.array([[math.cos(yaw),     -math.sin(yaw),    0],
                        [math.sin(yaw),     math.cos(yaw),     0],
                        [0,                 0,                 1]])
                        
        R = np.dot(R_z, np.dot( R_y, R_x ))
    
        return R