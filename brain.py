import time
import sensors, pose_graph


class Brain:
    def __init__(self, world=None, perceive=False, label='X', rsu_labels=None) -> None:
        self.memory_manager = MemoryManager(label, rsu_labels)
        if perceive:
            if world is None:
                raise ValueError("Perception requires a world")
            self.perception_manager = PerceptionManager(world)

    def remember(self, data):
        self.memory_manager.memories[f'{time.time()}'] = data

    def recall(self, what, query=None):
        return self.memory_manager.get_recollection(what, query)

    def forget(self, what):
        if what == 'all':
            self.perception_manager.destroy_actors()
        # TODO: forget by target id (with logic)
        self.memory_manager.data = {}

    def check_sensor(self, sensor_type):
        if self.perception_manager is None:
            raise ValueError("Perception is not enabled")
        if sensor_type.lower() != 'lidar':
            raise NotImplementedError(f'Sensor type [{sensor_type}] not supported')

    def add_sensor(self, args, vehicle, sensor_type, vis=False):
        self.check_sensor(sensor_type)
        self.perception_manager.spawn_sensor(args, vehicle, sensor_type, self.memory_manager)
        if vis:
            self.perception_manager.start_vis(sensor_type)

    def get_sensor_data(self, sensor_type):
        self.check_sensor(sensor_type)
        return self.perception_manager.lidar_manager.pc

    def get_sensor_vis(self, sensor_type):
        self.check_sensor(sensor_type)
        return self.perception_manager.lidar_manager.vis
        
    def get_graph(self):
        return (self.memory_manager.graph_factors, 
                self.memory_manager.graph_values)
    
    def load_graph(self, data):
        graph = (data[0], data[1])
        content = data[2]
        self.memory_manager.merge_graph(graph, content)
        self.memory_manager.detect_all_loops()


class PerceptionManager:
    def __init__(self, world) -> None:
        self.world = world
        self.lidar_manager = None

    def spawn_sensor(self, args, vehicle, sensor_type, memory_manager):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager = sensors.LidarManager(args, self.world)
            self.lidar_manager.spawn_lidar(vehicle, memory_manager)

    def start_vis(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager.start_vis()

    def destroy_actors(self):
        self.lidar_manager.destroy_actors()


class MemoryManager(pose_graph.PoseGraphManager):
    def __init__(self, label, rsu_labels) -> None:
        super().__init__(label)
        self.set_loop_detector(pose_graph.ScanContextManager())

        self.rsu_labels = rsu_labels
        self.memories = {}

    def keyframe(self, data):
        self.update(data, self.rsu_labels)

    def get_recollection(self, what, query=None):
        if what == 'graph': # TODO: Improve this
            return self.get_communication_data(existing_nodes=query, get_content=True) # TODO: conditions where get_content is False
        elif what=='targetID': # TODO: Implement this
            return self.get_data_by_label(query)
        elif what=='nodes':
            return self.get_nodes()
        elif what==None:
            return "No query provided"
        else:
            raise NotImplementedError(f'Recollection of [{what}] not supported')

    def get_communication_data(self, existing_nodes=None, get_content=False):
        content_dict = self.loop_detector.get_communication_data(existing_nodes=existing_nodes) if get_content else None
        return (self.graph_factors, self.graph_values, content_dict)

    def get_nodes(self):
        return self.graph_directory.nodes
        
    # def get_data_by_label(self, label):
    #     associated_data = self.label_graph[label]
    #     pass
        
    # def add_node(self, node_idx, ptcloud):
    #     self.content[f'{node_idx}']['timestamp'] = time.time()
    #     self.content[f'{node_idx}']['point_cloud'] = ptcloud

    # def get_node(self, node_idx):
    #     return self.content[f'{node_idx}']
    
    # def get_nodes_before_timestamp(self, timestamp):
    #     nodes = []
    #     for key, value in self.content.items():
    #         if value['timestamp'] < timestamp:
    #             nodes.append(key)
    #     return nodes
    
    # def compare_content(self, content):
    #     pass

    