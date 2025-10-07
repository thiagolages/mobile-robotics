from abc import ABC, abstractmethod

class PathPlanner(ABC):
    def __init__(self, sim, robot_name):
        self.sim = sim
        self.robot_name = robot_name

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def run(self):
        pass