import random
import collections
from brain import Brain

class Entity:
    def __init__(self, id_num, manager) -> None:
        self.id_num = id_num
        self.manager = manager

    def set_label (self, label):
        self.label = label
        
    def listen(self, communication_dict):
        # Protect against self-communication
        sender_type, sender_id = communication_dict['sender'].split()
        if self.id_num == sender_id:
            print(f"{self} received its own data: {communication_dict['data']}")
            return
        
        # Communication logic
        if communication_dict['type'] == 'request':
            self.get_response(sender_id, sender_type, communication_dict['data'])
        elif communication_dict['type'] == 'response':
            self.handle_response(sender_id, sender_type, communication_dict['data'])
        elif communication_dict['type'] == 'post':
            print(f"{self} received post: {communication_dict['data']} from {communication_dict['sender']}")
            self.brain.remember(communication_dict['data'])
        else:
            print(f"{self} received unknown data: {communication_dict['data']} from {communication_dict['sender']}")

    def send_msg(self, recipient, recipient_type, msg_type, data):
        package = {'sender': f"{self}",
                    'type': msg_type,
                    'data': data}                   
        self.manager.post(recipient, recipient_type, package)   

    def get_response(self, requester_id, requester_type, data):
        print(f"{self} received request from {requester_type} {requester_id}")
        data['data'] = self.brain.recall(data['what'])
        self.send_msg(requester_id, requester_type.lower(),
                      'response', data)
        
    def handle_response(self, responder_id, responder_type, data):
        print(f"{self} received response: {type(data)} from {responder_id} {responder_type}")
        self.brain.remember(data['data'])
        if data['what']=='graph':
            self.brain.load_graph(data['data'])
    
    def __str__(self) -> str:
        return f"{self.__class__.__name__} {self.id_num}"


class RSU(Entity):
    def __init__(self, id_num, location, communication_range, neighbors: list, manager) -> None:
        super().__init__(id_num, manager)

        self.v2i_range = communication_range
        self.location = location

        self.brain = Brain(label='X')
        # self.brain = Brain(label='R')

        self.neighbors = neighbors
        self.vehicles_in_range = []

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def ping_vehicle(self, vehicle):
        if self.location.distance(vehicle.vehicle.get_location()) <= self.v2i_range:
            vehicle.handle_ping(self.id_num)
            if vehicle.id_num not in self.vehicles_in_range:
                self.vehicles_in_range.append(vehicle.id_num)
                self.send_msg(vehicle.id_num, 'vehicle', 
                               'request', {'what': 'graph'})
            return True
        return False
        
    def ping_vehicle_list(self, vehicle_list):
        vehicles_pinged = []
        for vehicle in vehicle_list:
            if self.ping_vehicle(vehicle):
                vehicles_pinged.append(vehicle.id_num)

        for vehicle_id in self.vehicles_in_range:
            if vehicle_id not in vehicles_pinged:
                print(f"{self} lost vehicle {vehicle_id}")
                self.vehicles_in_range.remove(vehicle_id)
                

class Vehicle(Entity):
    def __init__(self, id_num, world, args, manager) -> None:
        super().__init__(id_num, manager)

        self.world = world
        self.args = args
        self.brain = Brain(world, perceive=True, label='X')
        # self.brain = Brain(world, perceive=True, label='V')
        
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
        print(f"created {self.vehicle.type_id} at {ego_init.location}")

    def spawn_sensor(self, sensor_type, vis=False):
        self.brain.add_sensor(self.args, self.vehicle, sensor_type, vis)

    def handle_ping(self, rsu_id):
        print(f"{self} received ping from RSU {rsu_id}")
        if rsu_id not in self.in_range_rsu:
            if len(self.in_range_rsu) == self.in_range_rsu.maxlen:
                target = self.in_range_rsu.pop()
                data = self.brain.recall(what='targetID', query=target)
                self.send_msg(rsu_id, 'rsu', 
                               'post', data)
            self.in_range_rsu.appendleft(rsu_id)
            self.send_msg(rsu_id, 'rsu', 
                          'request', {'what': None})

    def destroy_actors(self):
        self.brain.forget('all')
        self.vehicle.destroy()


# TODO: Implement data packet
class DataPacket:
    def __init__(self) -> None:
        pass

    def __str__(self) -> str:
        return f"{self.__class__.__name__}"

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}"