import argparse
import numpy as np
from scipy import spatial

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
        sc = ptcloud2sc(ptcloud, self.shape, self.max_length)
        rk = sc2rk(sc)

        self.current_node_idx = node_idx
        self.ptclouds[node_idx] = ptcloud
        self.scancontexts[node_idx] = sc
        self.ringkeys[node_idx] = rk

    def getPtcloud(self, node_idx):
        return self.ptclouds[node_idx]

    def detectLoop(self):        
        exclude_recent_nodes = 30
        valid_recent_node_idx = self.current_node_idx - exclude_recent_nodes

        if(valid_recent_node_idx < 1):
            return None, None, None
        else:
            # step 1
            ringkey_history = np.array(self.ringkeys[:valid_recent_node_idx])
            ringkey_tree = spatial.KDTree(ringkey_history)

            ringkey_query = self.ringkeys[self.current_node_idx]
            _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=self.num_candidates)

            # step 2
            query_sc = self.scancontexts[self.current_node_idx]
            
            nn_dist = 1.0 # initialize with the largest value of distance
            nn_idx = None
            nn_yawdiff = None
            for ith in range(self.num_candidates):
                candidate_idx = nncandidates_idx[ith]
                candidate_sc = self.scancontexts[candidate_idx]
                dist, yaw_diff = distance_sc(candidate_sc, query_sc)
                if(dist < nn_dist):
                    nn_dist = dist
                    nn_yawdiff = yaw_diff
                    nn_idx = candidate_idx

            if(nn_dist < self.threshold):
                nn_yawdiff_deg = nn_yawdiff * (360/self.shape[1])
                return nn_idx, nn_dist, nn_yawdiff_deg # loop detected!
            else:
                return None, None, None
            

def ptcloud2sc(ptcloud, sc_shape, max_length):
    num_ring = sc_shape[0]
    num_sector = sc_shape[1]

    gap_ring = max_length/num_ring
    gap_sector = 360/num_sector
    
    enough_large = 500
    sc_storage = np.zeros([enough_large, num_ring, num_sector])
    sc_counter = np.zeros([num_ring, num_sector])
    
    for point in ptcloud.points:
        point_height = point[2] + 2.0 # for setting ground is roughly zero 
        
        idx_ring, idx_sector = pt2rs(point, gap_ring, gap_sector, num_ring, num_sector)
        
        if sc_counter[idx_ring, idx_sector] >= enough_large:
            continue
        sc_storage[int(sc_counter[idx_ring, idx_sector]), idx_ring, idx_sector] = point_height
        sc_counter[idx_ring, idx_sector] = sc_counter[idx_ring, idx_sector] + 1

    sc = np.amax(sc_storage, axis=0)
        
    return sc


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


def pt2rs(point, gap_ring, gap_sector, num_ring, num_sector):
    x = point[0]
    y = point[1]
    # z = point[2]
    
    if(x == 0.0):
        x = 0.001
    if(y == 0.0):
        y = 0.001
    
    theta = xy2theta(x, y)
    faraway = np.sqrt(x*x + y*y)
    
    idx_ring = np.divmod(faraway, gap_ring)[0]       
    idx_sector = np.divmod(theta, gap_sector)[0]

    if(idx_ring >= num_ring):
        idx_ring = num_ring-1 # python starts with 0 and ends with N-1
    
    return int(idx_ring), int(idx_sector)


def sc2rk(sc):
    return np.mean(sc, axis=1)


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
# References: 
# Argparser:
#   2020 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB): open3d_lidar.
# Scan Context:
#   gisbi-kim: PyICP-SLAM.  https://github.com/gisbi-kim/PyICP-SLAM/
####################################################################################################