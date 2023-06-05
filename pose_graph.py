import copy
import math
import time
import gtsam
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import networkx as nx
from scipy import spatial

# TODO: Merging pose graphs with loop detection
# TODO: VCS-inspired merging of data
# TODO: Distributed pose graph optimization

# TODO: GLOBAL REFERENCE FRAME: GPS/INSS? or can it be done implicitly via place recognition?

class PoseGraphManager:
    def __init__(self, label, detect_online=False, loop_check_interval=10) -> None:
        self.label = label
        self.detect_online = detect_online
        self.loop_check_interval = loop_check_interval
        self.count = 0

        self.prior_cov = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
        self.constraint_cov = np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1])
        self.odometry_cov = gtsam.noiseModel.Diagonal.Sigmas(self.constraint_cov)
        self.loop_cov = gtsam.noiseModel.Diagonal.Sigmas(self.constraint_cov)

        self.graph_factors = gtsam.NonlinearFactorGraph()
        self.graph_values = gtsam.Values()
        self.graph_directory = nx.Graph()

        self.current_pose = np.eye(4)
        self.icp_init = np.eye(4)

        self.previous_pc = None
        self.loop_detector = None

        self.add_prior()

    def set_loop_detector(self, loop_detector):
        self.loop_detector = loop_detector

    def get_communication_data(self):
        content_dict = self.loop_detector.get_communication_data()
        return (self.graph_factors, self.graph_values, content_dict)

    def get_nodes(self):
        return self.graph_directory.nodes
    
    def generate_symbol(self, previous=False):
        node = self.count - 1 if previous else self.count
        return gtsam.symbol(self.label, node)

    def add_prior(self):
        symbol = self.generate_symbol()
        self.graph_directory.add_node(symbol)
        self.graph_factors.add(
            gtsam.PriorFactorPose3(symbol, 
                                   gtsam.Pose3(self.current_pose), 
                                   self.prior_cov))
        self.graph_values.insert(symbol, gtsam.Pose3(self.current_pose))
        self.count += 1

    def add_odometry(self, odom_transform):
        symbol = self.generate_symbol()
        prev_symbol = self.generate_symbol(previous=True)
        self.graph_directory.add_node(symbol)
        self.graph_directory.add_edge(prev_symbol, symbol)
        self.graph_factors.add(
            gtsam.BetweenFactorPose3(prev_symbol, symbol, 
                                     gtsam.Pose3(odom_transform), 
                                     self.odometry_cov))
        self.graph_values.insert(symbol, gtsam.Pose3(self.current_pose))

    def add_loop(self, symbol, loop_symbol, loop_transform):
        self.graph_directory.add_edge(symbol, loop_symbol)
        self.graph_factors.add(
            gtsam.BetweenFactorPose3(loop_symbol, symbol, 
                                     gtsam.Pose3(loop_transform), 
                                     self.loop_cov))
        
    def optimize(self, from_symbol):
        print("Optimizing pose graph...")
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph_factors,
                                                      self.graph_values, 
                                                      gtsam.LevenbergMarquardtParams())
        self.graph_values = optimizer.optimize()

        if self.detect_online:
            pose = self.graph_values.atPose3(from_symbol)

            self.current_pose[:3, 3] = np.array([pose.x(), pose.y(), pose.z()])    # Translation
            self.current_pose[:3, :3] = pose.rotation().matrix()                   # Rotation

    def update(self, point_cloud):
        symbol = self.generate_symbol()
        print(f"Current node: {symbol_to_str(symbol)}")
        if self.count == 1:
            self.previous_pc = copy.deepcopy(point_cloud)
        pc_downsampled = point_cloud.uniform_down_sample(every_k_points=10)
        pc_previous_downsampled = self.previous_pc.uniform_down_sample(every_k_points=10)

        content = self.loop_detector.generate_content(pc_downsampled)
        self.loop_detector.addNode(symbol, content)

        # TODO: Improve odometry (EKF?)
        odom_transform = self.icp(pc_downsampled, pc_previous_downsampled)

        self.current_pose = np.matmul(self.current_pose, odom_transform)
        self.add_odometry(odom_transform)
        self.icp_init = odom_transform

        if(self.detect_online and self.count > 1 and self.count % self.loop_check_interval == 0):
            self.detect_loop(symbol)
        
        self.previous_pc = copy.deepcopy(point_cloud)
        self.count += 1

    def merge_graph(self, graph, content):
        graph_factors, graph_values = graph
        # TODO: smarter merging
        for idx in range(graph_factors.size()):
            factor = graph_factors.at(idx)
            symbol = factor.keys()[1] if isinstance(factor, gtsam.BetweenFactorPose3) else factor.keys()[0]
            pose = graph_values.atPose3(symbol)
            self.merge_node(symbol, pose)
            self.graph_factors.add(factor)

            try:
                self.loop_detector.addNode(symbol, content[symbol])                
            except KeyError:
                print(f"Key {symbol_to_str(symbol)} does not exist in content dict")
    
    def merge_node(self, symbol, pose):
        try: # TODO: improve logic
            self.graph_values.insert(symbol, pose)
        except RuntimeError:
            print(f"Key {symbol_to_str(symbol)} already exists in graph")

    def close_loop(self, symbol_1, symbol_2, yaw_diff_deg):
        symbol_1_str = symbol_to_str(symbol_1)
        symbol_2_str = symbol_to_str(symbol_2)
        print(f'Loop detected between node {symbol_1_str} and {symbol_2_str}')
        pc_downsampled_1 = self.loop_detector.getPtcloud(symbol_1)
        pc_downsampled_2 = self.loop_detector.getPtcloud(symbol_2)
        loop_transform = self.icp(pc_downsampled_1, pc_downsampled_2,
                                    init_pose=self.yawdeg2se3(yaw_diff_deg))
        self.add_loop(symbol_1, symbol_2, loop_transform)
        self.optimize(symbol_2)

    def detect_loop(self, symbol):
        loop_symbol, _, yaw_diff_deg = self.loop_detector.detectLoop(symbol)
        if loop_symbol is not None:
            self.close_loop(symbol, loop_symbol, yaw_diff_deg)

    def detect_all_loops(self):
        loops = self.loop_detector.detectAllLoops()
        for loop_symbol, loop_candidates in loops.items():
            for loop_candidate in loop_candidates:
                print(f"loop_candidate: {symbol_to_str(loop_candidate[0])}, \
                      dist: {loop_candidate[1]}, yaw_diff: {loop_candidate[2]}")
                self.close_loop(loop_symbol, loop_candidate[0], loop_candidate[1])

    def icp(self, pc_downsampled, pc_previous_downsampled, init_pose=None):
        reg_p2p = o3d.pipelines.registration.registration_icp(source = pc_downsampled, 
                                                              target = pc_previous_downsampled, 
                                                              max_correspondence_distance = 10, 
                                                              init = self.icp_init if init_pose is None else init_pose, 
                                                              estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
                                                              criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
        return reg_p2p.transformation
    
    def yawdeg2se3(self, yaw_deg):
        yaw = np.deg2rad(yaw_deg)
        se3 = np.eye(4)
        se3[:3, :3] = np.array([[math.cos(yaw),     -math.sin(yaw),    0],
                                [math.sin(yaw),     math.cos(yaw),     0],
                                [0,                 0,                 1]]) 
        return se3    

class ScanContextManager:
    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): # defualt configs are same as the original paper 
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold
        self.max_length = 80 # recommended but other (e.g., 100m) is also ok.

        self.content_map = {}

    def get_communication_data(self):
        return self.content_map
    
    def generate_content(self, ptcloud):
        return {
            'pointcloud': ptcloud,
            'scancontext': self.ptcloud2sc(ptcloud),
            'ringkey': self.sc2rk(self.ptcloud2sc(ptcloud)),
            'timestamp': time.time()
        }

    def addNode(self, symbol, content):
        if symbol in self.content_map:
            self.mergeNode(symbol, content)
        else:
            self.content_map[symbol] = content

    def mergeNode(self, symbol, content): 
        # TODO: VCS merge data
        if self.content_map[symbol]['timestamp'] < content['timestamp']:
            self.content_map[symbol] = content

    def getPtcloud(self, symbol):
        return self.content_map[symbol]['pointcloud']

    def detectLoop(self, candidate_symbol):  
        # TODO: pull out node validation to a separate function so that merged and non-merged nodes can be handled there 
        node_content = self.content_map[candidate_symbol]

        exclude_nodes_since = 15 # seconds
        valid_before = node_content['timestamp'] - exclude_nodes_since

        symbol_history = np.array([symbol for symbol in self.content_map.keys()
                                    if self.content_map[symbol]['timestamp'] < valid_before])
        ringkey_history = np.array([self.content_map[symbol]['ringkey'] for symbol in symbol_history])

        if len(ringkey_history) == 0:
            return None, None, None
        else:
            print(f"Detecting loop for node {symbol_to_str(candidate_symbol)}...")   
            # step 1          
            ringkey_tree = spatial.KDTree(ringkey_history)

            ringkey_query = node_content['ringkey'] 
            _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

            # step 2
            query_sc = node_content['scancontext']
            
            nn_dist = 1.0 # initialize with the largest value of distance
            nn_symbol = None
            nn_yawdiff = None
            for ith in range(min(self.num_candidates, len(ringkey_history))):
                candidate_idx = nncandidates_idx[ith]
                candidate_symbol = symbol_history[candidate_idx]
                candidate_sc = self.content_map[candidate_symbol]['scancontext']
                dist, yaw_diff = self.distance_sc(candidate_sc, query_sc)
                if(dist < nn_dist):
                    nn_dist = dist
                    nn_yawdiff = yaw_diff
                    nn_symbol = symbol_history[candidate_idx]

            print(f"-----> nn_dist: {nn_dist}, nn_yawdiff: {nn_yawdiff}")
            if(nn_dist < self.threshold):
                nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
                return nn_symbol, nn_dist, nn_yawdiff_deg # loop detected!
            else:
                return None, None, None
            
    def detectAllLoops(self):
        loops = {}
        for symbol in self.content_map.keys():
            loop_symbol, loop_dist, loop_yawdeg = self.detectLoop(symbol)
            if(loop_symbol is not None):
                if(loop_symbol not in loops):
                    loops[loop_symbol] = []
                loops[loop_symbol].append((symbol, loop_dist, loop_yawdeg))
        return loops
            
    def ptcloud2sc(self, ptcloud):        
        enough_large = 500
        sc_storage = np.zeros([enough_large, self.shape[0], self.shape[1]])
        sc_counter = np.zeros([self.shape[0], self.shape[1]])
        
        for point in ptcloud.points:
            point_height = point[2]
            
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
        
        idx_ring = np.sqrt(x*x + y*y) // gap_ring      
        idx_sector = self.xy2theta(x, y) // gap_sector

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
                if (not np.count_nonzero(col_j_1) or not np.count_nonzero(col_j_2)):
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

    def plotNode(self, node_idx):
        fig = plt.figure(figsize=(20, 10))
        ax2 = fig.add_subplot()

        # plot scan context map
        sc = self.scancontexts[node_idx]
        ax2.imshow(sc, cmap='gray')

        plt.show()
        input()


def symbol_to_str(symbol):
    return f"{chr(gtsam.symbolChr(symbol))}{gtsam.symbolIndex(symbol)}"


# TODO: TBD if this is needed
class OdometryManager:
    def __init__(self) -> None:
        pass

####################################################################################################
# Adapted from:
#   gisbi-kim: PyICP-SLAM.  https://github.com/gisbi-kim/PyICP-SLAM/
####################################################################################################