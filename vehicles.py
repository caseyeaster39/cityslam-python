import random
import sensors

class Vehicle:
    def __init__(self, id_num, world, args, location, communication_range, manager) -> None:
        self.world = world
        self.lidar_manager = sensors.LidarManager(args, self.world)
        
        self.vehicle = None  
        self.id_num = id_num
        self.location = location
        self.communication_range = communication_range
        self.manager = manager

    def spawn_vehicle(self, vehicle_type, behavior, location=None):        
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_type)[0]
        ego_init = random.choice(self.world.get_map().get_spawn_points()) if location is None else location
        self.vehicle = self.world.spawn_actor(bp, ego_init)
        if behavior == 'autopilot':
            self.vehicle.set_autopilot(True)
        else:
            print('Behavior not supported')
        print('created %s' % self.vehicle.type_id) 

    def spawn_sensor(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            self.add_lidar()
        else:
            print('Sensor type not supported')

    def add_lidar(self):
        self.lidar_manager.spawn_lidar(self.vehicle)

    def start_vis(self):
        self.lidar_manager.start_vis()

    def post_data(self, manager, rsu, data):
        manager.post(rsu, data)

    def destroy_actors(self):
        self.lidar_manager.vis.destroy_window()
        self.lidar_manager.lidar.destroy()
        self.vehicle.destroy()

    def get_vehicle(self):
        return self.vehicle