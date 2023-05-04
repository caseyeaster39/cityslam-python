import random
import collections
from brain import Brain

class Entity:
    def __init__(self, id_num, manager):
        self.id_num = id_num
        self.manager = manager
        
    def listen(self, communication_dict):
        # Protect against self-communication
        if self.id_num == communication_dict['sender']:
            print(f"{self.__class__.__name__} {self.id_num} received its own data: {communication_dict['data']}")
            return
        
        # Communication logic
        if communication_dict['type'] == 'request':
            print(f"{self.__class__.__name__} {self.id_num} received request from entity {communication_dict['sender']}")
            self.post_data(communication_dict['sender'], communication_dict['sender_type'], {'sender': self.id_num,
                                                                                             'sender_type': self.__class__.__name__.lower(),
                                                                                             'type': 'response',
                                                                                             'data': f'response'})
        elif communication_dict['type'] == 'response':
            print(f"{self.__class__.__name__} {self.id_num} received response: {communication_dict['data']} from entity {communication_dict['sender']}")
            self.brain.remember(communication_dict['data'])
        elif communication_dict['type'] == 'post':
            print(f"{self.__class__.__name__} {self.id_num} received post: {communication_dict['data']} from entity {communication_dict['sender']}")
            self.brain.remember(communication_dict['data'])
        else:
            print(f"{self.__class__.__name__} {self.id_num} received unknown data: {communication_dict['data']} from entity {communication_dict['sender']}")

    def post_data(self, recepient, recepient_type, data):
        self.manager.post(recepient, recepient_type, data)

    def __str__(self):
        return f"{self.__class__.__name__} {self.id_num} at {self.location} with neighbors {self.neighbors}"


class RSU(Entity):
    def __init__(self, id_num, location, communication_range, neighbors: list, manager) -> None:
        super().__init__(id_num, manager)

        self.v2i_range = communication_range
        self.location = location

        self.brain = Brain()

        self.neighbors = neighbors
        self.vehicles_in_range = []

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def ping_vehicle(self, vehicle):
        if self.location.distance(vehicle.vehicle.get_location()) <= self.v2i_range:
            vehicle.handle_ping(self.id_num)
            if vehicle.id_num not in self.vehicles_in_range:
                self.vehicles_in_range.append(vehicle.id_num)
                self.post_data(vehicle.id_num, 'vehicle', {'sender': self.id_num, 
                                                           'sender_type': 'rsu', 
                                                           'type': 'request', 
                                                           'data': 'request'})
            return True
        return False
        
    def ping_vehicle_list(self, vehicle_list):
        vehicles_pinged = []
        for vehicle in vehicle_list:
            if self.ping_vehicle(vehicle):
                vehicles_pinged.append(vehicle.id_num)

        for vehicle_id in self.vehicles_in_range:
            if vehicle_id not in vehicles_pinged:
                print(f"RSU {self.id_num} lost vehicle {vehicle_id}")
                self.vehicles_in_range.remove(vehicle_id)
                

class Vehicle(Entity):
    def __init__(self, id_num, world, args, manager) -> None:
        super().__init__(id_num, manager)

        self.world = world
        self.args = args
        self.brain = Brain(world, perceive=True)
        
        self.vehicle = None
        self.in_range_rsu = collections.deque(maxlen=args.rsu_queue_size)

    def spawn_vehicle(self, vehicle_type, behavior, spawn_location=None):      
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_type)[0]
        ego_init = random.choice(self.world.get_map().get_spawn_points()) if spawn_location is None else spawn_location
        self.vehicle = self.world.spawn_actor(bp, ego_init)
        if behavior == 'autopilot':
            self.vehicle.set_autopilot(True)
        else:
            raise NotImplementedError("Behavior not supported")
        print(f"created {self.vehicle.type_id} at {ego_init}")

    def spawn_sensor(self, sensor_type, vis=False):
        self.brain.add_sensor(self.args, self.vehicle, sensor_type, vis)

    def handle_ping(self, rsu_id):
        print(f"Vehicle {self.id_num} received ping from RSU {rsu_id}")
        if rsu_id not in self.in_range_rsu:
            if len(self.in_range_rsu) == self.in_range_rsu.maxlen:
                target = self.in_range_rsu.pop()
                data = self.brain.recall(target)
                self.post_data(rsu_id, 'rsu', {'sender': self.id_num,
                                               'sender_type': 'vehicle',
                                               'type': 'post',
                                               'data': data})
            self.in_range_rsu.appendleft(rsu_id)
            self.post_data(rsu_id, 'rsu', {'sender': self.id_num,
                                           'sender_type': 'vehicle',
                                           'type': 'request',
                                           'data': f'request'})

    def destroy_actors(self):
        self.brain.forget('all')
        self.vehicle.destroy()