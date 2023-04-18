import random

class VehicleManager:
    def __init__(self, world) -> None:
        self.world = world
        self.vehicle = None  

    def spawn_vehicle(self, vehicle_type, behavior, location=None):        
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter(vehicle_type)[0]
        ego_init = random.choice(self.world.get_map().get_spawn_points()) if location is None else location
        self.vehicle = self.world.spawn_actor(bp, ego_init)
        if behavior == 'autopilot':
            self.vehicle.set_autopilot(True)
        else:
            print('Behavior not supported')
        print('created %s' % self.vehicle.type_id)   