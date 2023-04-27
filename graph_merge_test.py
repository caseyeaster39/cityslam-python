import gtsam
import random
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy


def value_safe_add(graph, symbol, pose):
    try:
        graph.insert(symbol, pose)
    except RuntimeError:
        print(f"Key {chr(gtsam.symbolChr(symbol))}{gtsam.symbolIndex(symbol)} already exists in graph")


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


def merge_pose_graphs(graph1, graph2):
    noise_model = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    # Create a new empty graph to merge the two graphs into
    merged_graph = gtsam.NonlinearFactorGraph()
    merged_graph_values = gtsam.Values()

    # Copy all factors and variables from the first graph into the merged graph
    merged_graph = graph1[0].clone()
    merged_graph_values = deepcopy(graph1[1])

    # Copy all factors and variables from the second graph into the merged graph,
    # and connect the two graphs on shared nodes
    for idx in range(graph2[0].size()):
        factor = graph2[0].at(idx)
        if isinstance(factor, gtsam.PriorFactorPose3):
            symbol = factor.keys()[0]
            pose = graph2[1].atPose3(symbol)
            value_safe_add(merged_graph_values, symbol, pose)
            merged_graph.add(gtsam.PriorFactorPose3(symbol, pose, noise_model))
        elif isinstance(factor, gtsam.BetweenFactorPose3):
            symbol1 = factor.keys()[0]
            symbol2 = factor.keys()[1]
            pose = factor.measured()
            value_safe_add(merged_graph_values, symbol1, pose)
            value_safe_add(merged_graph_values, symbol2, pose)
            merged_graph.add(gtsam.BetweenFactorPose3(symbol1, symbol2, pose, noise_model))
    return (merged_graph, merged_graph_values)


def optimize_pose_graph(graph):        
    # Optimize the merged graph and return the optimized estimate
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph[0], 
                                                  graph[1], 
                                                  gtsam.LevenbergMarquardtParams())
    result = optimizer.optimize()
    return (graph[0], result)


# TODO: Get rid of code redundancy between the first and second graph generators
def generate_pose_graphs(num_nodes, shared_nodes):
    noise_model = gtsam.noiseModel.Diagonal.Sigmas([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    # Generate random poses and odometry factors for the first graph
    graph1 = gtsam.NonlinearFactorGraph()
    graph1_values = gtsam.Values()
    
    for i in range(num_nodes):
        node_name = 's' if i in shared_nodes else 'a'
        if i == 0:
            pose = gtsam.Pose3()
            graph1.add(gtsam.PriorFactorPose3(gtsam.symbol(node_name, 0), pose, noise_model))
            graph1_values.insert(gtsam.symbol(node_name, 0), pose)
        else:
            odometry = gtsam.Pose3(gtsam.Rot3.Rodrigues(random.uniform(-0.1, 0.1), 
                                                        random.uniform(-0.1, 0.1), 
                                                        random.uniform(-0.1, 0.1)), 
                                                        gtsam.Point3(random.uniform(-0.1, 0.1), 
                                                                    random.uniform(-0.1, 0.1), 
                                                                    random.uniform(-0.1, 0.1)))
            pose = gtsam.Pose3(np.matmul(pose.matrix(), odometry.matrix()))
            graph1.add(gtsam.BetweenFactorPose3(gtsam.symbol(previous_node_name, i-1),
                                                gtsam.symbol(node_name, i),
                                                odometry, noise_model))
            graph1_values.insert(gtsam.symbol(node_name, i), pose)
        previous_node_name = node_name
    
    # Generate random poses and odometry factors for the second graph,
    # copying over some poses from the first graph to share nodes
    graph2 = gtsam.NonlinearFactorGraph()
    graph2_values = gtsam.Values()
    previous_node_name = None
    for i in range(num_nodes):
        if i in shared_nodes:  
            node_name = 's'     
            pose = graph1_values.atPose3(gtsam.symbol(node_name, i))
            if i == 0:
                graph2.add(gtsam.PriorFactorPose3(gtsam.symbol(node_name, i), 
                                                  pose, noise_model))
            else:
                graph2.add(gtsam.BetweenFactorPose3(gtsam.symbol(previous_node_name, i-1), 
                                                    gtsam.symbol(node_name, i), 
                                                    pose, noise_model))
        else:
            node_name = 'b'
            if i == 0:
                pose = generate_random_pose3()
                graph2.add(gtsam.PriorFactorPose3(gtsam.symbol(node_name, i),
                                                  pose, noise_model))
            else:
                odometry = gtsam.Pose3(gtsam.Rot3.Rodrigues(random.uniform(-0.1, 0.1), 
                                                            random.uniform(-0.1, 0.1), 
                                                            random.uniform(-0.1, 0.1)), 
                                                            gtsam.Point3(random.uniform(-0.1, 0.1), 
                                                                         random.uniform(-0.1, 0.1), 
                                                                         random.uniform(-0.1, 0.1)))
                graph2.add(gtsam.BetweenFactorPose3(gtsam.symbol(previous_node_name, i-1),
                                                    gtsam.symbol(node_name, i),
                                                    odometry, noise_model))
                pose = gtsam.Pose3(np.matmul(pose.matrix(), odometry.matrix()))
        graph2_values.insert(gtsam.symbol(node_name, i), pose)
        previous_node_name = node_name

    return (graph1, graph1_values), (graph2, graph2_values)

def generate_random_rotation3():
    # Generate a random quaternion using numpy
    quat = np.random.randn(4)
    quat /= np.linalg.norm(quat)

    # Convert the quaternion to a rotation matrix using gtsam
    w, x, y, z = quat
    rotation = gtsam.Rot3.Quaternion(w, x, y, z)

    return rotation

def generate_random_pose3(max_range=1):
    # Generate a random rotation using gtsam
    rotation = generate_random_rotation3()

    # Generate a random translation using gtsam
    translation = gtsam.Point3(
        x=random.uniform(-max_range, max_range),
        y=random.uniform(-max_range, max_range),
        z=random.uniform(-max_range*0.1, max_range*0.1)
    )

    # Combine the rotation and translation into a pose
    pose = gtsam.Pose3(rotation, translation)

    return pose


if __name__ == "__main__":
    plt.ion()

    # Generate two random pose graphs
    graph1, graph2 = generate_pose_graphs(10, [0, 1, 2, 3, 5])

    # Visualize the two graphs
    visualize_pose_graph(graph1, "Graph 1")
    visualize_pose_graph(graph2, "Graph 2")

    # Merge the two graphs and visualize the result
    merged_graph = merge_pose_graphs(graph1, graph2)
    visualize_pose_graph(merged_graph, "Merged Graph")

    # Optimize the merged graph and visualize the result
    optimized_graph = optimize_pose_graph(merged_graph)
    visualize_pose_graph(optimized_graph, "Optimized Graph")
    input()