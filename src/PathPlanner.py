
from abc import ABC, abstractmethod

class PathPlanner(ABC):
    def __init__(self, sim, robot_name, objects):
        self.sim = sim
        self.robot_name = robot_name
        self.objects = objects

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def run(self):
        pass

class Roadmap(PathPlanner):
    def __init__(self, sim, robot_name, objects, vehicle_type, map_name):
        super().__init__(sim, robot_name, objects)
        self.vehicle_type = vehicle_type
        self.map_name = map_name

    def setup(self):
        pass
    
    def run(self):
        pass

class PotentialField(PathPlanner):
    def __init__(self, sim, robot_name, objects, vehicle_type, map_name):
        super().__init__(sim, robot_name, objects)
        self.vehicle_type = vehicle_type
        self.map_name = map_name

    def setup(self):
        self.load_map()
        self.build_potential_field()
    
    def load_map(self):
        pass
    
    def build_potential_field(self):
        pass

    def run(self):
        pass

class RRT(PathPlanner):
    def __init__(self, sim, robot_name, objects, vehicle_type, map_name):
        super().__init__(sim, robot_name, objects)
        self.vehicle_type = vehicle_type
        self.map_name = map_name

    def setup(self):
        pass

    def run(self):
        pass