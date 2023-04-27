class RSU:
    def __init__(self, id_num, location, communication_range, neighbors: list, manager) -> None:
        self.id_num = id_num
        self.v2i_range = communication_range
        self.location = location
        self.neighbors = neighbors
        self.manager = manager

    def listen(self, data_dict: dict):
        # NOTE: be careful calling post from here, it could cause an infinite loop if its own id is not checked
        print(f"RSU {self.id_num} received data: {data_dict['type']} from entity {data_dict['sender']}")

    def post(self, id_num, data):
        self.manager.post(id_num, data)

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def ping_vehicle(self, vehicle):
        if self.location.distance(vehicle.vehicle.get_location()) <= self.v2i_range:
            vehicle.handle_ping(self.id_num)
            return True
        return False