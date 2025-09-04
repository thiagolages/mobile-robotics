import time

import numpy as np

from Assingment import Assignment
from HokuyoSensorSim import HokuyoSensorSim
from utils import (
    build_object_dict,
    get_tf,
    generate_random_pose,
    handle_sim_start,
    plot_sensor_data,
    set_pose,
    plot_robot_to_object_tfs,
)


class TP1(Assignment):
    def __init__(self, verbose=False):
        super().__init__(sleep=False, auto_start=False, stepping=True)
        self._setup()
        self.verbose = verbose

    def _setup(self):
        self.robot_name = "PioneerP3DX"
        self.robot_name = "PioneerP3DX"
        self.excluded_objects = [
            self.robot_name, # Exclude robot from objects
            self.robot_name, # Exclude robot from objects
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
            self.sim, self.laser_handle, verbose=self.verbose
        )  # 4 x 4 matrix
        self.T_laser_w = get_tf(
            self.sim, self.laser_handle, inv=True, verbose=self.verbose
        )  # 4 x 4 matrix

    def get_robot_tfs(self, verbose=False):
        self.T_w_robot = get_tf(
            self.sim, self.robot_handle, verbose=self.verbose
        )  # 4 x 4 matrix
        self.T_robot_w = get_tf(
            self.sim, self.robot_handle, inv=True, verbose=self.verbose
        )  # 4 x 4 matrix

    def get_robot_to_object_tfs(self):
        self.robot_to_obj_tfs = {}
        for name, obj in self.objects.items():
            handle = obj["handle"]
            T_w_obj = get_tf(self.sim, handle, verbose=self.verbose)  # 4 x 4 matrix
            # T_w_obj = T_w_robot @ T_robot_obj
            # -> left multiply by (T_w_robot)^-1, or T_robot_w
            # (T_w_robot)^-1 @ T_w_obj = T_robot_obj
            # T_robot_obj = T_robot_w @ T_w_obj
            T_robot_obj = self.T_robot_w @ T_w_obj
            self.robot_to_obj_tfs[name] = T_robot_obj
            if self.verbose:
                print(f"Transform from robot to {name}: \n{T_robot_obj}")
                print("-" * 40)

    def setupExercises(self):
        handle_sim_start(self.sim)
        self.robot_initial_pose_list = self.sim.getObjectPose(self.robot_handle)
        self.robot_initial_pose = self.sim.poseToMatrix(self.robot_initial_pose_list)
        self.get_robot_tfs(verbose=True)
        self.get_laser_tfs(verbose=True)
        self.get_robot_to_object_tfs()

    def _executing_exc(self, exc_number):
        print("-" * 40)
        print(f"Executing exercise {exc_number}")
        print("-" * 40)

    def _finished_exc(self, exc_number):
        pass
        # print("-" * 40)
        # print(f"Finished exercise {exc_number}")
        # print("-" * 40)

    def exc(self, id):
        self._executing_exc(id)
        if hasattr(self, f"exc{id}"):
            self.__getattribute__(f"exc{id}")()
        else:
            raise ValueError(f"Exercise {id} not found")
        self._finished_exc(id)

    def exc4(self):
        for idx in range(1, 5, 1):
            self.sim.step()
            pose = generate_random_pose(
                self.sim,
                xlim=[-1, 1],
                ylim=[-1, 1],
                zlim=[self.T_w_robot[2, 3], self.T_w_robot[2, 3]],  # curr Z
                theta_lim=[0, 2 * np.pi],
            )
            if self.verbose:
                print(f"Random pose {idx}: \n{pose}")
            set_pose(self.sim, self.robot_handle, pose)
            self.get_robot_tfs()
            self.get_robot_to_object_tfs()
            plot_robot_to_object_tfs(
                self.robot_name,
                self.robot_to_obj_tfs,
                camera_angle=(50, 70, -15),  # elev, azim, roll
                save_path=f"plots/{idx}_plot.png",
                title=f"Robot-to-Object Transforms, pose {idx}",
                verbose=self.verbose,
            )

    def exc5(self):
        pass


    def exc6(self):
        self.P_world_array = None
        count = 1
        interval = 3 # seconds
        max_iter = 10 # number of iterations
        current_time = self.sim.getSimulationTime()
        initial_time = time.time()

        # Reset robot pose
        print(f"Robot initial pose:\n {self.robot_initial_pose}")
        initial_pose = np.array(self.robot_initial_pose).reshape(3, 4)
        print(f"Robot initial pose reshaped:\n {initial_pose}")
        initial_pose = np.vstack((initial_pose, np.array([[0, 0, 0, 1]])))
        print(f"Robot initial pose reshaped and stacked:\n {initial_pose}")
        print(f"Resetting robot pose to\n {initial_pose}")
        set_pose(self.sim, self.robot_handle, pose=self.robot_initial_pose)
        self.sim.step()
        self.get_robot_tfs(verbose=self.verbose)
        print(f"Robot current pose:\n {self.T_w_robot}")

        print("Getting sensor data every {} seconds".format(interval))
        while count <= max_iter:
            self.sim.step()
            # self.sim.wait(0.05)
            # time.sleep(0.05)
            # print("Stepping")
            time_passed = self.sim.getSimulationTime() - current_time
            # print(f"Time passed: {time_passed}")
            if time_passed >= interval:
                current_time = self.sim.getSimulationTime()
                # self.sim.pauseSimulation()
            else:
                continue

            print(f"Getting {self.laser_name} sensor data at timestep {count}.")
            # Transform from laser to the world (inverse of world to laser)
            self.get_robot_tfs(verbose=self.verbose)
            self.get_laser_tfs(verbose=self.verbose)

            self.T_robot_laser = self.T_robot_w @ self.T_w_laser

            # Get raw data as 3D points in sensor frame
            P_laser = self.hokuyo_sensor.getSensorData()
            # Normalize vectors
            P_laser = np.hstack(
                [np.array(P_laser), np.ones((len(P_laser), 1))]
            )
            # Transpose to get 4 x N
            P_laser = P_laser.T

            # Transform to world frame
            P_world = self.T_w_robot @ self.T_robot_laser @ P_laser

            if self.P_world_array is None:
                self.P_world_array = P_world
            else:
                self.P_world_array = np.concatenate(
                    (self.P_world_array, P_world), axis=1
                )

            count += 1

    def run(self):
        try:
            # 0 - Setup Exercises
            self.setupExercises()

            # Questions 3 and 4
            # 3 - Plot the transforms from the robot to all objects in the scene
            # 4 - Repeat question 3 for 3x other random poses
            self.exc(4)

            # Question 5
            # Add a laser to the robot;
            # Define transform T_r_l (robot as reference, laser being described)
            # Define transform T_w_r (world as reference, robot being described)
            # Plot the laser points with respect to the world frame
            # p_w = T_w_l @ p_l
            self.exc(5)        

            # Question 6
            # Make an incremental plot showing the path executed by the robot
            # (like a dashed line), and all the combined laser readings along
            # the way. They should be plotted in the world frame.
            self.exc(6)

            # Pause simulation
            print("Pausing simulation")
            self.sim.pauseSimulation()
        
            self.ax, self.fig = plot_sensor_data(self.P_world_array, ax=self.ax, show=True, block=True)

            while True:
                self.sim.step()
                time.sleep(1)
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            self.sim.stopSimulation()
        

if __name__ == "__main__":
    tp1 = TP1(verbose=False)
    tp1.run()
