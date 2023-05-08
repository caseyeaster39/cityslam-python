import copy
import math
import gtsam
import numpy as np
import open3d as o3d
from scipy import spatial

# TODO: Importing pose graphs
# TODO: Merging pose graphs
# TODO: Distributed pose graph optimization
# TODO: Vehicle vs. RSU pose graphs


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

    def add_loop(self, loop_idx_1, loop_idx_2, loop_transform):
        self.graph_factors.add(
            gtsam.BetweenFactorPose3(gtsam.symbol('x', loop_idx_2), 
                                     gtsam.symbol('x', loop_idx_1), 
                                     gtsam.Pose3(loop_transform), 
                                     self.loop_cov))
        
    def optimize(self):
        print("Optimizing pose graph...")
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph_factors,
                                                      self.graph_values, 
                                                      gtsam.LevenbergMarquardtParams())
        self.graph_optimized = optimizer.optimize()

        pose = self.graph_optimized.atPose3(gtsam.symbol('x', self.current_node_idx))

        self.current_pose[:3, 3] = np.array([pose.x(), pose.y(), pose.z()])    # Translation
        self.current_pose[:3, :3] = pose.rotation().matrix()                   # Rotation

    def update(self, point_cloud):
        print(f"Current node: {self.current_node_idx}")
        if self.current_node_idx == 1:
            self.previous_pc = copy.deepcopy(point_cloud)
        pc_downsampled = point_cloud.uniform_down_sample(every_k_points=10)
        pc_previous_downsampled = self.previous_pc.uniform_down_sample(every_k_points=10)

        self.loop_detector.addNode(node_idx=self.current_node_idx-1,
                                   ptcloud=pc_downsampled)

        # TODO: Improve odometry (EKF?)
        odom_transform = self.icp(pc_downsampled, pc_previous_downsampled)

        self.current_pose = np.matmul(self.current_pose, odom_transform)
        self.add_odometry(odom_transform)
        self.icp_init = odom_transform

        # if(self.curr_node_idx > 1 and self.curr_node_idx % 10 == 0):
        #     self.detect_loops(self.current_node_idx-1)
        
        self.previous_pc = copy.deepcopy(point_cloud)
        self.previous_node_idx = self.current_node_idx
        self.current_node_idx += 1

    def detect_loops(self, node_idx):
        loop_idx, _, yaw_diff_deg = self.loop_detector.detectLoop(node_idx)
        if loop_idx is not None:
            self.close_loop(node_idx, loop_idx, yaw_diff_deg)

    def close_loop(self, loop_idx_1, loop_idx_2, yaw_diff_deg):
        print(f'Loop detected between node {loop_idx_1} and {loop_idx_2}')
        pc_downsampled_1 = self.loop_detector.getPtcloud(loop_idx_1)
        pc_downsampled_2 = self.loop_detector.getPtcloud(loop_idx_2)
        loop_transform = self.icp(pc_downsampled_1, pc_downsampled_2,
                                    init_pose=self.yawdeg2se3(yaw_diff_deg))
        self.add_loop(loop_idx_1+1, loop_idx_2+1, loop_transform)
        self.optimize()

    def detect_all_loops(self):
        loops = self.loop_detector.detectAllLoops()
        for loop_idx, loop_candidates in loops.items():
            for loop_candidate in loop_candidates:
                print(f"loop_candidate: {loop_candidate}")
                self.close_loop(loop_idx, loop_candidate[0], loop_candidate[1])

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
        se3[:3, :3] = self.eulerAnglesToRotationMatrix(yaw_rad)
        return se3 

    @staticmethod
    def eulerAnglesToRotationMatrix(yaw) :        
        # assuming roll and pitch both equal 0
        
        # yaw
        R = np.array([[math.cos(yaw),     -math.sin(yaw),    0],
                        [math.sin(yaw),     math.cos(yaw),     0],
                        [0,                 0,                 1]])    
        return R
    

class ScanContextManager:
    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): # defualt configs are same as the original paper 
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold

        self.max_length = 80 # recommended but other (e.g., 100m) is also ok.

        storage_maximum = 15000 # capable of up to [storage_maximum] number of nodes 
        self.ptclouds = [None] * storage_maximum
        self.scancontexts = [None] * storage_maximum
        self.ringkeys = [None] * storage_maximum

        self.current_node_idx = 0

    def addNode(self, node_idx, ptcloud):
        sc = self.ptcloud2sc(ptcloud)
        rk = self.sc2rk(sc)

        self.current_node_idx = node_idx
        self.ptclouds[node_idx] = ptcloud
        self.scancontexts[node_idx] = sc
        self.ringkeys[node_idx] = rk

    def getPtcloud(self, node_idx):
        return self.ptclouds[node_idx]

    def detectLoop(self, node_idx):     
        print(f"Detecting loop for node {node_idx}...")   
        exclude_recent_nodes = 30
        valid_recent_node_idx = node_idx - exclude_recent_nodes

        if(valid_recent_node_idx < 1):
            return None, None, None
        else:
            # step 1
            ringkey_history = np.array(self.ringkeys[:valid_recent_node_idx])
            ringkey_tree = spatial.KDTree(ringkey_history)

            ringkey_query = self.ringkeys[node_idx]
            _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

            # step 2
            query_sc = self.scancontexts[node_idx]
            
            nn_dist = 1.0 # initialize with the largest value of distance
            nn_idx = None
            nn_yawdiff = None
            for ith in range(self.num_candidates):
                candidate_idx = nncandidates_idx[ith]
                candidate_sc = self.scancontexts[candidate_idx]
                dist, yaw_diff = self.distance_sc(candidate_sc, query_sc)
                if(dist < nn_dist):
                    nn_dist = dist
                    nn_yawdiff = yaw_diff
                    nn_idx = candidate_idx

            print(f"-----> nn_dist: {nn_dist}, nn_yawdiff: {nn_yawdiff}")
            if(nn_dist < self.threshold):
                nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
                return nn_idx, nn_dist, nn_yawdiff_deg # loop detected!
            else:
                return None, None, None
            
    def detectAllLoops(self):
        loops = {}
        # TODO: parallelize this loop detection process
        # TODO: clustering for search efficiency
        for node_idx in range(self.current_node_idx): # TODO: better way to do this range
            loop_idx, _, loop_yawdeg = self.detectLoop(node_idx)
            if(loop_idx is not None):
                if(node_idx not in loops.keys()):
                    loops[node_idx] = []
                loops[node_idx].append([loop_idx, loop_yawdeg])

        return loops

            
    def ptcloud2sc(self, ptcloud):        
        enough_large = 500
        sc_storage = np.zeros([enough_large, self.shape[0], self.shape[1]])
        sc_counter = np.zeros([self.shape[0], self.shape[1]])
        
        for point in ptcloud.points:
            point_height = point[2] + 2.0 # for setting ground is roughly zero 
            
            idx_ring, idx_sector = self.pt2rs(point)
            
            if sc_counter[idx_ring, idx_sector] >= enough_large:
                continue
            sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
            sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1

        sc = np.amax(sc_storage, axis=0)
            
        return sc


    def pt2rs(self, point):
        gap_ring = self.max_length/self.shape[0]
        gap_sector = 360/self.shape[1]
        x = point[0]
        y = point[1]
        # z = point[2]
        
        if(x == 0.0):
            x = 0.001
        if(y == 0.0):
            y = 0.001
        
        theta = self.xy2theta(x, y)
        faraway = np.sqrt(x*x + y*y)
        
        idx_ring = np.divmod(faraway, gap_ring)[0]       
        idx_sector = np.divmod(theta, gap_sector)[0]

        if(idx_ring >= self.shape[0]):
            idx_ring = self.shape[0]-1 # python starts with 0 and ends with N-1
        
        return int(idx_ring), int(idx_sector)

    @staticmethod
    def xy2theta(x, y):
        if (x >= 0 and y >= 0): 
            theta = 180/np.pi * np.arctan(y/x)
        if (x < 0 and y >= 0): 
            theta = 180 - ((180/np.pi) * np.arctan(y/(-x)))
        if (x < 0 and y < 0): 
            theta = 180 + ((180/np.pi) * np.arctan(y/x))
        if ( x >= 0 and y < 0):
            theta = 360 - ((180/np.pi) * np.arctan((-y)/x))

        return theta

    @staticmethod
    def sc2rk(sc):
        return np.mean(sc, axis=1)

    @staticmethod
    def distance_sc(sc1, sc2):
        num_sectors = sc1.shape[1]

        # repeate to move 1 columns
        _one_step = 1 # const
        sim_for_each_cols = np.zeros(num_sectors)
        for i in range(num_sectors):
            # Shift
            sc1 = np.roll(sc1, _one_step, axis=1) #  columne shift

            #compare
            sum_of_cossim = 0
            num_col_engaged = 0
            for j in range(num_sectors):
                col_j_1 = sc1[:, j]
                col_j_2 = sc2[:, j]
                if (~np.any(col_j_1) or ~np.any(col_j_2)): 
                    # to avoid being divided by zero when calculating cosine similarity
                    # - but this part is quite slow in python, you can omit it.
                    continue 

                cossim = np.dot(col_j_1, col_j_2) / (np.linalg.norm(col_j_1) * np.linalg.norm(col_j_2))
                sum_of_cossim = sum_of_cossim + cossim

                num_col_engaged = num_col_engaged + 1

            # save 
            sim_for_each_cols[i] = sum_of_cossim / num_col_engaged

        yaw_diff = np.argmax(sim_for_each_cols) + 1 # because python starts with 0 
        sim = np.max(sim_for_each_cols)
        dist = 1 - sim

        return dist, yaw_diff

####################################################################################################
# Adapted from:
#   gisbi-kim: PyICP-SLAM.  https://github.com/gisbi-kim/PyICP-SLAM/
####################################################################################################