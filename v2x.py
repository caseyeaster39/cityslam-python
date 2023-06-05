import uuid 
import entities

class V2X_Manager:
    def __init__(self, world, args) -> None:
        self.world = world
        self.args = args

        self.rsu_list = []
        self.vehicle_list = []

        self.rsu_counter = 0
        self.vehicle_counter = 0

    def get_label(self, entity_type):
        if entity_type == 'rsu':
            response = chr(ord('A') + self.rsu_counter)
            self.rsu_counter += 1
        elif entity_type == 'vehicle':
            response = chr(ord('a') + self.vehicle_counter)
            self.vehicle_counter += 1
        else:
            raise ValueError(f"{entity_type} is not a valid entity type")
        return response

    def add_rsu(self, spawn_location, communication_range, neighbors: list):
        label = self.get_label('rsu')
        rsu = entities.RSU(uuid.uuid4(), spawn_location, communication_range, neighbors, self, label)
        self.rsu_list.append(rsu)

    def add_vehicle(self, vehicle_type, behavior, spawn_location=None, sensors = [], vis=False):
        label = self.get_label('vehicle')
        vehicle = entities.Vehicle(uuid.uuid4(), self.world, self.args, self, label)
        vehicle.spawn_vehicle(vehicle_type, behavior, spawn_location)
        if len(sensors) > 0:
            for sensor in sensors:
                vehicle.spawn_sensor(sensor_type=sensor, vis=vis)
        self.vehicle_list.append(vehicle)

    def get_entity(self, id_num, entity_type):
        if entity_type == 'rsu':
            entity_list = self.rsu_list 
        elif entity_type == 'vehicle':
            entity_list = self.vehicle_list
        else:
            raise ValueError(f"{entity_type} is not a valid entity type")
        
        for entity in entity_list:
            if str(entity.id_num) == str(id_num):
                return entity
        raise ValueError(f"{entity_type} with id_num {id_num} does not exist")
    
    def post(self, recepient, recepient_type, data):
        self.get_entity(recepient, recepient_type).listen(data)

    def ping(self):
        for rsu in self.rsu_list:
            rsu.ping_vehicle_list(self.vehicle_list)
                    