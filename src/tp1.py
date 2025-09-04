import time

import numpy as np

from Assingment import Assignment
from HokuyoSensorSim import HokuyoSensorSim
from utils import (
    build_object_dict,
    get_tf,
    handle_sim_start,
    plot_sensor_data,
)


class TP1(Assignment):
    def __init__(self):
        super().__init__(sleep=False, auto_start=False, stepping=True)
        self._setup()

    def _setup(self):
        self.excluded_objects = [
            "DefaultCamera",
            "XYZCameraProxy",
            "DefaultLights",
            "Floor",
        ]
        self.ax = None
        self.robot_name = "/PioneerP3DX"
        self.laser_name = "/fastHokuyo"
        self.laser_full_name = self.robot_name + self.laser_name
        self.robot_handle = self.sim.getObject(self.robot_name)
        self.laser_handle = self.sim.getObject(self.laser_full_name)

        self.objects = build_object_dict(self.sim, self.excluded_objects)

        self.hokuyo_sensor = HokuyoSensorSim(self.sim, self.laser_full_name)
        self.hokuyo_sensor.set_is_range_data(
            False
        )  # Get points in laser frame

    def get_laser_tfs(self, verbose=False):
        self.T_w_laser = get_tf(
            self.sim, self.laser_handle, verbose=verbose
        )  # 4 x 4 matrix
        self.T_laser_w = get_tf(
            self.sim, self.laser_handle, inv=True, verbose=verbose
        )  # 4 x 4 matrix

    def get_robot_tfs(self, verbose=False):
        self.T_w_robot = get_tf(
            self.sim, self.robot_handle, verbose=verbose
        )  # 4 x 4 matrix
        self.T_robot_w = get_tf(
            self.sim, self.robot_handle, inv=True, verbose=verbose
        )  # 4 x 4 matrix

    def get_robot_to_object_tfs(self, verbose=False):
        self.robot_to_obj_tfs = {}
        for name, obj in self.objects.items():
            handle = obj["handle"]
            T_w_obj = get_tf(self.sim, handle, verbose=verbose)  # 4 x 4 matrix
            # T_w_obj = T_w_robot @ T_robot_obj
            # -> left multiply by (T_w_robot)^-1, or T_robot_w
            # (T_w_robot)^-1 @ T_w_obj = T_robot_obj
            # T_robot_obj = T_robot_w @ T_w_obj
            T_robot_obj = self.T_robot_w @ T_w_obj
            self.robot_to_obj_tfs[name] = T_robot_obj
            if verbose:
                print(f"Transform from robot to {name}: \n{T_robot_obj}")
                print("-" * 40)

    def run(self):
        # 0 - Setup
        sim = self.sim
        handle_sim_start(sim)
        self.get_robot_tfs(verbose=True)
        self.get_laser_tfs(verbose=True)
        self.get_robot_to_object_tfs(verbose=True)

        # Questions 3 and 4
        # 3 - Plot the transforms from the robot to all objects in the scene
        # 4 - Repeat question 3 for 3x other random poses

        # for idx in range(4):
        #     pose = generate_random_pose(
        #         sim,
        #         xlim=[-1, 1],
        #         ylim=[-1, 1],
        #         zlim=[self.T_w_robot[2, 3], self.T_w_robot[2, 3]],  # curr Z
        #         theta_lim=[0, 2 * np.pi],
        #     )
        #     print(f"Random pose {idx}: \n{pose}")
        #     set_pose(sim, self.robot_handle, pose)
        #     self.get_robot_tfs()
        #     self.get_robot_to_object_tfs()
        #     plot_robot_to_object_tfs(
        #         self.robot_name,
        #         self.robot_to_obj_tfs,
        #         camera_angle=(50, 70, -15),  # elev, azim, roll
        #         save_path=f"plots/{idx}_plot.png",
        #     )

        # Question 5
        # Add a laser to the robot;
        # Define transform T_r_l (robot as reference, laser being described)
        # Define transform T_w_r (world as reference, robot being described)
        # Plot the laser points with respect to the world frame
        # p_w = T_w_l @ p_l
        self.P_world_array = None
        count = 0
        max_iter = 10
        current_time = self.sim.getSimulationTime()
        initial_time = time.time()

        while count < max_iter:
            self.sim.step()
            self.sim.wait(0.05)
            time.sleep(0.05)
            print("Stepping")
            if self.sim.getSimulationTime() - current_time >= 5:
                current_time = self.sim.getSimulationTime()
                print(f"Getting {count} sensor data.")
                # self.sim.pauseSimulation()
            else:
                continue

            print("Getting sensor data")
            # Transform from laser to the world (inverse of world to laser)
            self.get_robot_tfs(verbose=False)
            self.get_laser_tfs(verbose=False)

            self.T_robot_laser = self.T_robot_w @ self.T_w_laser
            # print(f"T_robot_laser: \n {self.T_robot_laser}")
            # print("-" * 40)
            # Get raw data as 3D points in sensor frame
            P_laser = self.hokuyo_sensor.getSensorData()
            # Normalize vectors
            P_laser = np.hstack(
                [np.array(P_laser), np.ones((len(P_laser), 1))]
            )
            # Transpose to get 4 x N
            P_laser = P_laser.T
            # print(f"P_laser: \n {P_laser}")
            # print(f"P_laser.shape: \n {P_laser.shape}")
            # print("-" * 40)
            # Transform to world frame
            P_world = self.T_w_robot @ self.T_robot_laser @ P_laser
            # print(f"P_world: \n {P_world}")
            # print(f"P_world.shape: \n {P_world.shape}")
            # print("-" * 40)
            # print("First point in laser frame: \n", P_laser[:, 0].T)
            # print("First point in robot frame: \n", self.T_robot_laser @ P_laser[:, 0].T)
            # print("First point in world frame: \n", P_world[:, 0].T)
            # print("-" * 40)
            # print("Average of Z points: \n", np.mean(P_world[2, :]))
            # print("-" * 40)
            if self.P_world_array is None:
                self.P_world_array = P_world
            else:
                self.P_world_array = np.concatenate(
                    (self.P_world_array, P_world), axis=1
                )

            count += 1
            print("Finished getting sensor data")

            # self.sim.startSimulation()

        self.ax = plot_sensor_data(self.P_world_array, ax=self.ax, show=True)

        # Question 6
        # Make an incremental plot showing the path executed by the robot
        # (like a dashed line), and all the combined laser readings along
        # the way. They should be plotted in the world frame.


if __name__ == "__main__":
    tp1 = TP1()
    tp1.run()
