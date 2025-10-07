
from abc import ABC, abstractmethod

class Controller(ABC):
    def __init__(self, sim, robot_name):
        self.sim = sim
        self.robot_name = robot_name

    @abstractmethod
    def run(self, waypoints=None):
        """Run the controller to follow waypoints.
        
        Args:
            waypoints: List of waypoints to follow. Each waypoint should be [x, y] or [x, y, theta]
        """
        pass