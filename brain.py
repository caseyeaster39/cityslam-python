import sensors, pose_graph, utils_ref


class Brain:
    def __init__(self, world, args) -> None:
        self.world = world
        self.args = args

        # TODO: store data in a data manager
        self.memory_manager = MemoryManager()
        self.perception_manager = PerceptionManager(self.world)

    def remember(self, data):
        self.memory_manager.data = data

    def recall(self):
        return self.memory_manager.data

    def forget(self, what):
        if what == 'all':
            self.perception_manager.destroy_actors()
        self.memory_manager.data = {}

    def add_sensor(self, args, vehicle, sensor_type, vis=False):
        self.perception_manager.spawn_sensor(args, vehicle, sensor_type, self.memory_manager)
        if vis:
            self.perception_manager.start_vis(sensor_type)

    def get_sensor_data(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            return self.perception_manager.lidar_manager.pc
        else:
            raise ValueError(f'Sensor type [{sensor_type}] not supported')

    def get_sensor_vis(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            return self.perception_manager.lidar_manager.vis
        else:
            raise ValueError(f'Visualization for [{sensor_type}] not supported')


class PerceptionManager:
    def __init__(self, world) -> None:
        self.world = world
        self.lidar_manager = None

    def spawn_sensor(self, args, vehicle, sensor_type, memory_manager):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager = sensors.LidarManager(args, self.world)
            self.lidar_manager.spawn_lidar(vehicle, memory_manager)   
        else:
            raise ValueError(f'Sensor type [{sensor_type}] not supported')

    def start_vis(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            self.lidar_manager.start_vis()
        else:
            raise ValueError(f'Visualization for [{sensor_type}] not supported')

    def destroy_actors(self):
        self.lidar_manager.destroy_actors()


class MemoryManager:
    def __init__(self) -> None:
        self.pose_graph_manager = pose_graph.PoseGraphManager()
        self.pose_graph_manager.add_prior()
        self.pose_graph_manager.set_loop_detector(utils_ref.ScanContextManager())
        self.data = {}

    def keyframe(self, data):
        self.pose_graph_manager.update(data)

    