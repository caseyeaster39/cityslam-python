class RSU:
    def __init__(self, id_num, location, communication_range, neighbors: list, manager) -> None:
        self.id_num = id_num
        self.v2i_range = communication_range
        self.location = location
        self.neighbors = neighbors
        self.manager = manager

    def listen(self, communication_dict: dict):
        if self.id_num == communication_dict['sender']:
            print(f"RSU {self.id_num} received its own data: {communication_dict['data']}")
            return
        if communication_dict['type'] == 'request':
            print(f"RSU {self.id_num} received request: {communication_dict['data']} from entity {communication_dict['sender']}")
            self.post(communication_dict['sender'], communication_dict['sender_type'], {'sender': self.id_num,
                                                                      'sender_type': 'rsu',
                                                                      'type': 'response',
                                                                      'data': f'response from {self.id_num}'})
        elif communication_dict['type'] == 'response':
            print(f"RSU {self.id_num} received response: {communication_dict['data']} from entity {communication_dict['sender']}")
            self.brain.remember(communication_dict['data'])
        elif communication_dict['type'] == 'post':
            print(f"RSU {self.id_num} received post: {communication_dict['data']} from entity {communication_dict['sender']}")
            self.brain.remember(communication_dict['data'])
        else:
            print(f"RSU {self.id_num} received unknown data: {communication_dict['data']} from entity {communication_dict['sender']}")

    def post(self, recepient, recepient_type, data):
        self.manager.post(recepient, recepient_type, data)

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def ping_vehicle(self, vehicle):
        if self.location.distance(vehicle.vehicle.get_location()) <= self.v2i_range:
            return True
        return False