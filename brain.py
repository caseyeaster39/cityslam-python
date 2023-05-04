import time
import sensors, pose_graph, utils_ref


class Brain:
    def __init__(self, world=None, perceive=False) -> None:
        self.memory_manager = MemoryManager()
        if perceive:
            if world is None:
                raise ValueError("Perception requires a world")
            self.perception_manager = PerceptionManager(world)

    def remember(self, data):
        self.memory_manager.data[f'{time.time()}'] = data

    def recall(self, what):
        return self.memory_manager.get_recollection(what)

    def forget(self, what):
        if what == 'all':
            self.perception_manager.destroy_actors()
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
        return (self.memory_manager.pose_graph_manager.graph_factors, 
                self.memory_manager.pose_graph_manager.graph_values)


class PerceptionManager:
    def __init__(self, world) -> None:
        self.world = world
        self.lidar_manager = None

    def spawn_sensor(self, args, vehicle, sensor_type, memory_manager):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager = sensors.LidarManager(args, self.world)
            self.lidar_manager.spawn_lidar(vehicle, memory_manager)   
        else:
            raise NotImplementedError(f'Sensor type [{sensor_type}] not supported')

    def start_vis(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager.start_vis()
        else:
            raise NotImplementedError(f'Visualization for [{sensor_type}] not supported')

    def destroy_actors(self):
        self.lidar_manager.destroy_actors()


class MemoryManager:
    def __init__(self) -> None:
        self.pose_graph_manager = pose_graph.PoseGraphManager()
        self.pose_graph_manager.add_prior()
        self.pose_graph_manager.set_loop_detector(utils_ref.ScanContextManager())

        self.label_graph = {}

        self.data = {}

    def keyframe(self, data):
        self.pose_graph_manager.update(data)
        # Data snapshot, label, etc

    def get_recollection(self, what):
        if what == 'all':
            return self.data
        elif what == 'graph':
            return self.pose_graph_manager.graph_factors
        else:
            raise NotImplementedError(f'Recollection of [{what}] not supported')

    