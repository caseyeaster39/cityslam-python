import uuid 
import roadside_units, vehicles

class V2X_Manager:
    def __init__(self, world, args) -> None:
        self.world = world
        self.args = args

        self.rsu_list = []
        self.vehicle_list = []

    def add_rsu(self, location, communication_range, neighbors: list):
        rsu = roadside_units.RSU(uuid.uuid4(), location, communication_range, neighbors, self)
        self.rsu_list.append(rsu)

    def add_vehicle(self, vehicle_type, behavior, location=None, communication_range=50, sensors = [], vis=False):
        vehicle = vehicles.Vehicle(uuid.uuid4(), self.world, self.args, location, communication_range, self)
        vehicle.spawn_vehicle(vehicle_type, behavior, location)
        if len(sensors) > 0:
            for sensor in sensors:
                vehicle.spawn_sensor(sensor_type=sensor)
        self.vehicle_list.append(vehicle)
        if vis:
            vehicle.start_vis()

    def get_rsu(self, id_num):
        for rsu in self.rsu_list:
            if rsu.id_num == id_num:
                return rsu
        return None
    
    def post(self, id_num, data):
        rsu = self.get_rsu(id_num)
        if rsu is None:
            raise ValueError(f"RSU with id_num {id_num} does not exist")
        rsu.listen(data)
    
    