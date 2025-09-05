import time
import os

from matplotlib import pyplot as plt
import numpy as np

# Configure matplotlib for interactive, non-blocking plots
plt.ion()  # Enable interactive mode

from Assingment import Assignment
from HokuyoSensorSim import HokuyoSensorSim
from utils import (
    build_object_dict,
    get_tf,
    tf_from_pose,
    generate_random_tf,
    handle_sim_start,
    plot_sensor_data,
    set_pose,
    plot_robot_to_object_tfs,
)

# Configure numpy to display matrices with at most 2 decimal places
np.set_printoptions(precision=2, suppress=True)

class TP1(Assignment):
    def __init__(self, verbose=False):
        super().__init__(sleep=False, auto_start=False, stepping=True)
        self.verbose = verbose
        self._setup()

    def _setup(self):
        
        self.ax = None
        self.num_random_poses = 3
        self.robot_random_tfs = []
        self.P_laser_random_poses = []
        self.plt_perspective_view = (60, -130, -15)  # elev, azim, roll
        self.plt_top_view = (90, -90, 0)  # elev, azim, roll

        self.plot_x_range = [-3, 3]
        self.plot_y_range = [-3, 3]
        self.plot_z_range = [0, 1]

        self.robot_name = "/PioneerP3DX"
        self.laser_name = "/fastHokuyo"
        self.excluded_objects = [
            self.robot_name, # Exclude robot from objects
            "DefaultCamera",
            "XYZCameraProxy",
            "DefaultLights",
            "Floor",
            "robot_frame",
            "world_frame"
        ]
        self.robot_handle = self.sim.getObject(self.robot_name)
        self.laser_handle = self.sim.getObject(self.laser_name)

        self.objects = build_object_dict(
            self.sim, 
            self.excluded_objects, 
            verbose=self.verbose
        )

        self.hokuyo_sensor = HokuyoSensorSim(self.sim, self.laser_name)
        self.hokuyo_sensor.set_is_range_data(
            False
        )  # Get points in laser frame

    def setupExercises(self):
        
        # Create plots directory if it doesn't exist
        os.makedirs("plots", exist_ok=True)
        
        handle_sim_start(self.sim)
        
        # Save initial robot pose
        self.robot_initial_pose_list = (
            self.sim.getObjectPose(self.robot_handle)
        )
        self.robot_initial_pose = (
            self.sim.poseToMatrix(self.robot_initial_pose_list)
        )
        
        # Get current robot, laser and objects poses
        self.get_robot_tfs(verbose=self.verbose)
        self.get_laser_tfs(verbose=self.verbose)
        self.get_robot_to_object_tfs(verbose=self.verbose)

        # Since this transform is static, it needs to be calculated only once
        self.T_robot_laser = self.T_robot_w @ self.T_w_laser

        # Get data used by some exercises
        self.get_random_tfs_and_sensor_data(num_poses=self.num_random_poses)


    def get_random_tfs_and_sensor_data(self, num_poses=3):
        
        for idx in range(num_poses):
            tf = generate_random_tf(
                self.sim,
                xlim=[-1, 1],
                ylim=[-1, 1],
                zlim=[self.T_w_robot[2, 3], self.T_w_robot[2, 3]],  # curr Z
                theta_lim=[0, 2 * np.pi],
            )
            self.robot_random_tfs.append(tf)
            print(f"Generated random tf {idx}: \n{tf}")

            # Set tf and step sim
            set_pose(self.sim, self.robot_handle, tf)
            self.sim.step()

            # Never print transforms here
            self.get_robot_tfs(verbose=False)

            # Get sensor data
            self.P_laser_random_poses.append(
                self.hokuyo_sensor.getSensorData()
            ) 

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

    def get_robot_to_object_tfs(self, verbose=False):
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
            if verbose:
                print(f"Transform from robot to {name}: \n{T_robot_obj}")
                print("-" * 40)

    def _executing_exc(self, exc_number):
        print("#" * 40)
        print(f"Executing exercise {exc_number}")
        print("#" * 40)

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

    def exc3(self):
        """
        Question 3
        Plot the transforms from the robot to all objects in the scene.
        """
        # Always print transforms here
        self.get_robot_tfs(verbose=True)
        self.get_robot_to_object_tfs(verbose=True)

        # Keep sim going
        self.sim.step()
        
        # Plot with non-blocking display
        fig, ax = plot_robot_to_object_tfs(
            self.robot_name,
            self.robot_to_obj_tfs,
            T_w_robot=self.T_w_robot,
            camera_angle=self.plt_top_view,
            save_path=f"plots/q3.png",
            title=f"Exc 3 plot, Top View",
            verbose=False,
        )

    def exc4(self):
        """
        Question 4
        Repeat question 3 for 3x other random poses.
        """
        for idx in range(1, self.num_random_poses + 1, 1):
            pose = self.robot_random_tfs[idx-1]
            print(f"Generated random pose {idx}: \n{pose}")
            set_pose(self.sim, self.robot_handle, pose)
            
            # Always print transforms here
            self.get_robot_tfs(verbose=True)
            self.get_robot_to_object_tfs(verbose=True)

            # Keep sim going
            self.sim.step()
            
            # Plot TOP VIEW
            fig, ax = plot_robot_to_object_tfs(
                self.robot_name,
                self.robot_to_obj_tfs,
                T_w_robot=self.T_w_robot,
                camera_angle=self.plt_top_view,
                save_path=f"plots/q4_top_view_plot_{idx}.png",
                title=f"Exc 4 plot, Top View, pose {idx}",
                verbose=False,
            )

            # Plot PERSPECTIVE VIEW
            fig, ax = plot_robot_to_object_tfs(
                self.robot_name,
                self.robot_to_obj_tfs,
                T_w_robot=self.T_w_robot,
                camera_angle=self.plt_perspective_view,
                save_path=f"plots/q4_perspective_plot_{idx}.png",
                title=f"Exc 4 plot, Perspective View, pose {idx}",
                verbose=False,
                show=False,
            )
            
            # Small pause to allow the plot window to appear and update
            plt.pause(0.1)
            
        # Keep all plot windows open
        print("All plots generated. Plot windows should remain open.")
        plt.pause(0.5)  # Final pause to ensure all windows are displayed

    def exc5(self):
        """
        Question 5
        Add a laser to the robot;
        Define transform T_r_l (robot as reference, laser being described)
        Define transform T_w_r (world as reference, robot being described)
        Plot the laser points with respect to the world frame
        p_w = T_w_l @ p_l
        """
        for idx, (P_laser, T_w_robot) in enumerate(zip(self.P_laser_random_poses, self.robot_random_tfs)):
            
            # Keep sim going
            self.sim.step()

            # Normalize vectors
            P_laser = np.hstack(
                [np.array(P_laser), np.ones((len(P_laser), 1))]
            )
            # Transpose to get 4 x N
            P_laser = P_laser.T

            # Transform to world frame
            P_world = T_w_robot @ self.T_robot_laser @ P_laser

            ax, fig = plot_sensor_data(
                P_world, 
                ax=self.ax, 
                show=True, 
                block=False,
                save_path=f"plots/q5_plot_{idx+1}.png",
                range_dict={
                    "x": self.plot_x_range,
                    "y": self.plot_y_range,
                    "z": self.plot_z_range,
                },
                robot_path=T_w_robot[:3, 3].tolist(),
            )
            plt.pause(0.5)



    def exc6(self):
        """
        Question 6
        Make an incremental plot showing the path executed by the robot
        (like a dashed line), and all the combined laser readings along
        the way. They should be plotted in the world frame.
        """
        self.P_world_array = None
        count = 1
        interval = 3 # seconds
        max_iter = 10 # number of iterations
        current_time = self.sim.getSimulationTime()
        self.robot_path = []

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
            
            # Store robot position for path tracking
            self.robot_path.append(self.T_w_robot[:3, 3].tolist())

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
        
        print(f"Collected sensor data from {count-1} timesteps")
        print(f"Robot path has {len(self.robot_path)} points")

        # Now plot the sensor data with robot path
        print("Plotting sensor data with robot path...")
        self.ax, self.fig = plot_sensor_data(
            self.P_world_array, 
            ax=self.ax, 
            show=True, 
            block=False,
            robot_path=self.robot_path,
            save_path=f"plots/q6.png",
        )
        
        # # Force the plot to display
        plt.draw()
        plt.pause(0.5)  # Longer pause to ensure the plot is fully rendered
            

    def run(self):
        try:
            # 0 - Setup Exercises
            self.setupExercises()

            # Question 1 - Create a scene with a PioneerP3DX mobile robot
            # and 5 different objects (scenes/TP1.ttt)
            
            # Question 2 - Define their reference frames and show the tfs
            # between them in a diagram (plots/q2.png)

            # Question 3 - Plot the transforms from the robot to all objects
            # in the scene
            self.exc(3)

            # Question 4 - Repeat question 3 for 3x other random poses
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
            # print("Pausing simulation")
            # self.sim.pauseSimulation()
            # print("Sensor data plot displayed. Simulation paused.")
            # print("Press Ctrl+C to exit.")
            
            self.sim.stopSimulation()

            plt.show(block=True)  # blocks until you manually close all windows

            # while True:
            #     # Keep the program running and GUI responsive
            #     plt.pause(0.1)
            #     time.sleep(0.1)

        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            self.sim.stopSimulation()
            plt.close("all")  # Close all plot windows

if __name__ == "__main__":
    tp1 = TP1(verbose=False)
    tp1.run()
