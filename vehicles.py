import random
import collections
import sensors

class Vehicle:
    def __init__(self, id_num, world, args, manager) -> None:
        self.world = world
        self.args = args
        self.sensor_manager = sensors.SensorManager(self.world)
        # TODO: bring pose graph manager in here
        # TODO: bring loop detector in here
        # TODO: store data in a data manager
        
        self.vehicle = None  
        self.id_num = id_num
        self.in_range_rsu = collections.deque(maxlen=args.rsu_queue_size)
        self.manager = manager

        self.data = {}

    def spawn_vehicle(self, vehicle_type, behavior, spawn_location=None):      
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_type)[0]
        ego_init = random.choice(self.world.get_map().get_spawn_points()) if spawn_location is None else spawn_location
        self.vehicle = self.world.spawn_actor(bp, ego_init)
        if behavior == 'autopilot':
            self.vehicle.set_autopilot(True)
        else:
            print('Behavior not supported')
        print('created %s' % self.vehicle.type_id) 

    def spawn_sensor(self, sensor_type):
        if sensor_type.lower() == 'lidar':
            self.sensor_manager.spawn_lidar(self.args, self.vehicle)
        else:
            print('Sensor type not supported')

    def start_vis(self):
        self.sensor_manager.start_vis('lidar')

    def handle_ping(self, rsu_id):
        print(f"Vehicle {self.id_num} received ping from RSU {rsu_id}")
        if rsu_id not in self.in_range_rsu:
            if len(self.in_range_rsu) == self.in_range_rsu.maxlen:
                target = self.in_range_rsu.pop()
                data = self.get_rsu_data(target)
                self.post_data(rsu_id, {'sender': self.id_num,
                                        'type': 'post',
                                        'data': data})
            self.in_range_rsu.appendleft(rsu_id)
            self.post_data(rsu_id, {'sender': self.id_num,
                                    'type': 'request',
                                    'data': f'request from {self.id_num}'})
            
    def get_rsu_data(self, rsu):
        # TODO: actually, data in self.data will be labeled by rsu id, 
        # and there will be some data that is not rsu specific
        return self.data[rsu]

    def post_data(self, rsu, data):
        self.manager.post(rsu, data)

    def destroy_actors(self):
        self.sensor_manager.destroy_actors()
        self.vehicle.destroy()

    def get_vehicle(self):
        return self.vehicle