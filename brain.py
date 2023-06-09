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

    def recall(self, what, query=None, flag=False):
        return self.memory_manager.get_recollection(what, query, flag)

    def forget(self, what):
        if what == 'all':
            self.perception_manager.destroy_actors()
        if what == 'outdated':
            self.memory_manager.clear_outdated_memories()

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
        directory = data[3]
        self.memory_manager.merge_graph(graph, content, directory)


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
        self.memories = {} # TODO: Use this for VCS or delete?

    def keyframe(self, data):
        self.update(data, self.rsu_labels)

    def get_recollection(self, what, query=None, flag=False):
        if what == 'graph':
            return self.get_communication_data(existing_nodes=query, get_content=flag)
        elif what=='targetID':
            return self.get_data_by_label(query)
        elif what=='nodes':
            return self.get_nodes()
        elif what==None:
            return "No query provided"
        else:
            raise NotImplementedError(f'Recollection of [{what}] not supported')

    def get_communication_data(self, existing_nodes=None, get_content=False):
        content_dict = self.loop_detector.get_communication_data(existing_nodes=existing_nodes) if get_content else None
        return (self.graph_factors, self.graph_values, content_dict, self.graph_directory)

    def get_nodes(self):
        return self.graph_directory.nodes
        
    def get_data_by_label(self, label):
        response = {}
        for k, v in self.loop_detector.content_map.items():
            remove = True
            for rsu in v['rsus']:
                if rsu == label:
                    response[k] = v
                if rsu in self.rsu_labels:
                    remove = False
            self.loop_detector.content_map[k]['removal'] = remove
        return response        

    def clear_outdated_memories(self):
        keep = {k: self.loop_detector.content_map[k] for k in self.loop_detector.content_map.keys() 
                if self.loop_detector.content_map[k]['removal'] == False}
        self.loop_detector.content_map = keep
        
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

    