import random
import collections
from brain import Brain

class Vehicle:
    def __init__(self, id_num, world, args, manager) -> None:
        self.world = world
        self.args = args
        self.brain = Brain(self.world, self.args, perceive=True)
        
        self.vehicle = None  
        self.id_num = id_num
        self.in_range_rsu = collections.deque(maxlen=args.rsu_queue_size)
        self.manager = manager

    def spawn_vehicle(self, vehicle_type, behavior, spawn_location=None):      
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_type)[0]
        ego_init = random.choice(self.world.get_map().get_spawn_points()) if spawn_location is None else spawn_location
        self.vehicle = self.world.spawn_actor(bp, ego_init)
        if behavior == 'autopilot':
            self.vehicle.set_autopilot(True)
        else:
            raise NotImplementedError("Behavior not supported")
        print('created %s' % self.vehicle.type_id) 

    def spawn_sensor(self, sensor_type, vis=False):
        self.brain.add_sensor(self.args, self.vehicle, sensor_type, vis)

    def handle_ping(self, rsu_id):
        print(f"Vehicle {self.id_num} received ping from RSU {rsu_id}")
        if rsu_id not in self.in_range_rsu:
            if len(self.in_range_rsu) == self.in_range_rsu.maxlen:
                target = self.in_range_rsu.pop()
                data = self.get_rsu_data(target)
                self.post_data(rsu_id, 'rsu', {'sender': self.id_num,
                                               'sender_type': 'vehicle',
                                               'type': 'post',
                                               'data': data})
            self.in_range_rsu.appendleft(rsu_id)
            self.post_data(rsu_id, 'rsu', {'sender': self.id_num,
                                           'sender_type': 'vehicle',
                                           'type': 'request',
                                           'data': f'request from {self.id_num}'})
            
    def listen(self, data_dict: dict):
        if data_dict['type'] == 'request':
            print(f"Vehicle {self.id_num} received request: {data_dict['data']} from entity {data_dict['sender']}")
            self.post_data(data_dict['sender'], data_dict['sender_type'], {'sender': self.id_num,
                                                                           'sender_type': 'vehicle',
                                                                           'type': 'response',
                                                                           'data': f'response from {self.id_num}'})
        elif data_dict['type'] == 'response':
            print(f"Vehicle {self.id_num} received response: {data_dict['data']} from entity {data_dict['sender']}")
            self.brain.remember(data_dict['data'])
        else:
            print(f"Vehicle {self.id_num} received unknown data: {data_dict['data']} from entity {data_dict['sender']}")

    def post_data(self, recepient, recepient_type, data):
        self.manager.post(recepient, recepient_type, data)

    def destroy_actors(self):
        self.brain.forget('all')
        self.vehicle.destroy()

    def get_vehicle(self):
        return self.vehicle