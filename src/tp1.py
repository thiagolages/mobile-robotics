from Assignment import Assignment
from utils import (
    build_object_dict,
    get_tf,
    handle_sim_start,
    plot_robot_to_object_tfs,
)


class TP1(Assignment):
    def __init__(self):
        super().__init__(sleep=False, auto_start=False)
        self._setup()

    def _setup(self):
        self.excluded_objects = [
            "DefaultCamera",
            "XYZCameraProxy",
            "DefaultLights",
            "Floor",
        ]
        self.robot_name = "hexapod"
        self.objects = build_object_dict(self.sim, self.excluded_objects)
        self.robot_handle = self.sim.getObject("/" + self.robot_name)

    def get_robot_tfs(self):
        self.T_w_robot = get_tf(
            self.sim, self.robot_handle, verbose=True
        )  # 4 x 4 matrix
        self.T_robot_w = get_tf(
            self.sim, self.robot_handle, inv=True, verbose=True
        )  # 4 x 4 matrix

    def get_robot_to_object_tfs(self):
        self.robot_to_obj_tfs = {}
        for name, obj in self.objects.items():
            handle = obj["handle"]
            T_w_obj = get_tf(self.sim, handle, verbose=False)  # 4 x 4 matrix
            # T_w_obj = T_w_robot @ T_robot_obj
            # -> left multiply by (T_w_robot)^-1, or T_robot_w
            # (T_w_robot)^-1 @ T_w_obj = T_robot_obj
            # T_robot_obj = T_robot_w @ T_w_obj
            T_robot_obj = self.T_robot_w @ T_w_obj
            self.robot_to_obj_tfs[name] = T_robot_obj
            print(f"Transform from robot to {name}: {T_robot_obj}")
            print("-" * 40)

    def run(self):
        handle_sim_start(self.sim)
        self.get_robot_tfs()
        self.get_robot_to_object_tfs()
        plot_robot_to_object_tfs(self.robot_name, self.robot_to_obj_tfs)


if __name__ == "__main__":
    tp1 = TP1()
    tp1.run()
