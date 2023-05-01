import uuid 
import roadside_units, vehicles

class V2X_Manager:
    def __init__(self, world, args) -> None:
        self.world = world
        self.args = args

        self.rsu_list = []
        self.vehicle_list = []

    def add_rsu(self, spawn_location, communication_range, neighbors: list):
        rsu = roadside_units.RSU(uuid.uuid4(), spawn_location, communication_range, neighbors, self)
        self.rsu_list.append(rsu)

    def add_vehicle(self, vehicle_type, behavior, spawn_location=None, sensors = [], vis=False):
        vehicle = vehicles.Vehicle(uuid.uuid4(), self.world, self.args, self)
        vehicle.spawn_vehicle(vehicle_type, behavior, spawn_location)
        if len(sensors) > 0:
            for sensor in sensors:
                vehicle.spawn_sensor(sensor_type=sensor, vis=vis)
        self.vehicle_list.append(vehicle)

    def get_rsu(self, id_num):
        for rsu in self.rsu_list:
            if rsu.id_num == id_num:
                return rsu
        return None
    
    def get_vehicle(self, id_num):
        for vehicle in self.vehicle_list:
            if vehicle.id_num == id_num:
                return vehicle
        return None
    
    def post(self, from_entity, id_num, data):
        if from_entity == 'rsu': # TODO: this check should look for target entity type, since there is I2I and V2V communication
            to_entity = self.get_vehicle(id_num)
        elif from_entity == 'vehicle':
            to_entity = self.get_rsu(id_num)
        else:
            raise ValueError(f"{from_entity} is not a valid entity type")
        
        if to_entity is None:
            raise ValueError(f"{to_entity} with id_num {id_num} does not exist")
        to_entity.listen(data)

    def ping(self):
        for rsu in self.rsu_list:
            for vehicle in self.vehicle_list:
                if rsu.ping_vehicle(vehicle):
                    vehicle.handle_ping(rsu.id_num)
    
    