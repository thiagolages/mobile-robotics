import matplotlib

from Assingment import Assignment
from utils import (
    get_objects_handles,
    get_objects_poses,
)

matplotlib.use("TkAgg")  # Use TkAgg backend for interactive display


class TP1(Assignment):
    def __init__(self):
        super().__init__(sleep=False)
        self._setup()

    def _setup(self):
        self.excluded_objects = [
            "DefaultCamera",
            "XYZCameraProxy",
            "DefaultLights",
            "Floor",
        ]
        self.objects = get_objects_handles(self.sim, self.excluded_objects)
        self.objects = get_objects_poses(self.sim, self.objects, verbose=True)

    def run(self):
        pass


if __name__ == "__main__":
    tp1 = TP1()
    tp1.run()
