class RSU:
    def __init__(self, id_num, location, communication_range, neighbors: list, manager) -> None:
        self.id_num = id_num
        self.v2i_range = communication_range
        self.location = location
        self.neighbors = neighbors
        self.manager = manager

    def listen(self, data_dict: dict):
        # NOTE: be careful calling post from here, it could cause an infinite loop if its own id is not checked
        if data_dict['type'] == 'request':
            print(f"RSU {self.id_num} received request: {data_dict['data']} from entity {data_dict['sender']}")
            self.post(data_dict['sender'], data_dict['sender_type'], {'sender': self.id_num,
                                                                      'sender_type': 'rsu',
                                                                      'type': 'response',
                                                                      'data': f'response from {self.id_num}'})
        elif data_dict['type'] == 'response':
            print(f"RSU {self.id_num} received response: {data_dict['data']} from entity {data_dict['sender']}")

    def post(self, recepient, recepient_type, data):
        self.manager.post(recepient, recepient_type, data)

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def ping_vehicle(self, vehicle):
        if self.location.distance(vehicle.vehicle.get_location()) <= self.v2i_range:
            return True
        return False