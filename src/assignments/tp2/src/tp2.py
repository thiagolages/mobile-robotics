import os
import sys

# Get the absolute path to the src directory
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.append(src_path)

from Assignment import Assignment
from Roadmap import Roadmap
from PotentialField import PotentialField
from RRT import RRT
from HolonomicController import HolonomicController
from DifferentialController import DifferentialController

import numpy as np
from matplotlib import pyplot as plt

from utils import (
    get_tf,
    handle_sim_start,
)

# Ensure plots are rendered inline in the notebook
# %matplotlib inline
# Configure matplotlib for interactive, non-blocking plots
plt.ion()  # Enable interactive mode

# Configure numpy to display matrices with at most 2 decimal places
np.set_printoptions(precision=2, suppress=True)

class TP2(Assignment):
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
        self.T_w_robot = {}
        self.T_robot_w = {}
        self.map_names = ["map1", "map2"]
        self.vehicle_types = ["holonomic", "differential"]
        self.planning_algorithms = ["roadmap", "potential_fields", "rrt"]
        # Holonomic, differential
        self.robot_names = ["robotino", "PioneerP3DX"]
        self.robot_handles = [self.sim.getObject("/" + robot_name) for robot_name in self.robot_names]

        self.robot_name_mapping = {
            vehicle_type: robot_name
            for robot_name, vehicle_type in zip(self.robot_names, self.vehicle_types)
        }

        handle_sim_start(self.sim)
        self.get_robot_tfs()

    def get_robot_tfs(self):
        for robot_name, robot_handle in zip(self.robot_names, self.robot_handles):
            self.T_w_robot[robot_name] = get_tf(
                    self.sim, robot_handle, verbose=True
                )  # 4 x 4 matrix
            self.T_robot_w[robot_name] = get_tf(
                self.sim, robot_handle, inv=True, verbose=True
            )  # 4 x 4 matrix

    def setup_planner(self, planner_type, map_name, map_size, plot=False, seed=42, **kwargs):
        
        # Choose robot name based on planner type
        if planner_type == "roadmap":
            self.robot_name = self.robot_name_mapping["holonomic"] # Required by this assignment
            self.planner = Roadmap(
                sim=self.sim, 
                robot_name=self.robot_name,
                map_name=map_name, 
                map_size=map_size,
                plot=plot,
                **kwargs
            )
        elif planner_type == "potential_fields":
            self.robot_name = self.robot_name_mapping["differential"] # Required by this assignment
            self.planner = PotentialField(
                sim=self.sim, 
                robot_name=self.robot_name,
                map_name=map_name, 
                map_size=map_size,
                plot=plot,
                **kwargs
            )
        elif planner_type == "rrt":
            self.robot_name = self.robot_name_mapping["holonomic"] # Required by this assignment
            self.planner = RRT(
                sim=None,
                robot_name=self.robot_name,
                map_name=map_name,
                map_size=map_size, 
                plot=plot,
                seed=seed,
                **kwargs
            )
        else:
            raise ValueError(f"Invalid planner type: {planner_type}")
        
        # Setup selected planner
        self.planner.setup()

    def start_controller(self, waypoints=None, gain_x=0.5, gain_y=0.5, gain_theta=0.1):
        """Start the appropriate controller for the given vehicle type.
        
        Args:
            vehicle_type: Type of vehicle ("holonomic" or "differential")
            waypoints: List of waypoints to follow. Each waypoint should be [x, y] or [x, y, theta]
        """
        if isinstance(self.planner, Roadmap) or isinstance(self.planner, RRT):
            self.controller = HolonomicController(
                self.sim,
                self.robot_name,
                self.planner
            )
            self.controller.run(waypoints, gain_x, gain_y, gain_theta)

        elif isinstance(self.planner, PotentialField):
            self.controller = DifferentialController(
                self.sim,
                self.robot_name,
                self.planner
            )
            self.controller.run(self.planner.goal_pos)
        else:
            raise ValueError(f"Invalid planner type: {type(self.planner)}")


if __name__ == "__main__":
    try:
        # Setup TP2
        tp2 = TP2()
        plot = True
        seed = 42
        
        # 1) MAP 1
        map_name = "map1"
        map_size = (1000, 1000) # cols, rows, in pixels
        map_dims = (10, 10) # in meter
        cell_size = 0.25
        start = (-3.5, -3.5)
        goal = (3.5, 3.5)
        
        # 1.1) Roadmap
        planner_type = "roadmap"
        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            map_dims=map_dims,
            plot=plot,
            seed=seed,
            cell_size=cell_size,
        )
        path, simplified_path = tp2.planner.run(
            start=(-3.5, 0), # in meters
            goal=(3.5, 3.5), # in meters
            use_Astar=False,
            use_8_connected=False,
        )
        print(f"Path: {path}")
        print(f"Simplified Path: {simplified_path}")
        
        # Original Path
        tp2.start_controller(
            waypoints=path,
            gain_x=0.5,
            gain_y=0.5,
            gain_theta=0.1,
        )
        
        input("Press enter to run Roadmap (Grid) Simplified Path...")

        # Simplified Path
        tp2.start_controller(
            waypoints=simplified_path,
            gain_x=0.5,
            gain_y=0.5,
            gain_theta=0.1,
        )

        
        # 1.2) Potential Fields
        planner_type = "potential_fields"
        input("Press enter to setup Potential Fields...")
        
        goal_pos = (3.5, 3.5, 165) # x, y, theta

        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            plot=plot,
            goal_pos=goal_pos,
        )

        # This would be to plot
        # path = tp2.planner.run(
        #     start=(-3.5, -3.5), # in meters
        #     goal=(3.5, 3.5), # in meters,
        #     step_size=100.0, # in pixels
        #     max_iters=3000,
        #     goal_sample_rate=0.1, # percentage
        #     animate=True,
        #     collision_resolution=1, # in pixels
        # )

        tp2.start_controller()

        # 1.3) RRT
        input("Press enter to start planning with RRT...")
        planner_type = "rrt"
        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            plot=plot,
            seed=seed,
            map_dims=map_dims,
        )
        tree, edges, path_px, path = tp2.planner.run(
            start=(-3.5, -3.5), # in meters
            goal=(3.5, 3.5), # in meters,
            step_size=100.0, # in pixels
            max_iters=3000,
            goal_sample_rate=0.1, # percentage
            animate=True,
            collision_resolution=1, # in pixels
        )
        
        print(f"Path: {path}")
        input("Press enter to run RRT...")
        tp2.start_controller(waypoints=path, gain_x=0.5, gain_y=0.5, gain_theta=0.1)

        input("PLEASE SWITCH TO MAP 2 AND THEN PRESS ENTER TO CONTINUE...")

        # 2) MAP 2
        map_name = "map2"
        map_size = (1000, 1000) # cols, rows, in pixels
        map_dims = (10, 10) # in meter
        start = (-3.5, -3.5)
        goal = (3.5, 3.5)
        cell_size = 0.4

        # 2.1) Roadmap
        planner_type = "roadmap"
        # tp2.setup_planner(planner_type=planner_type, map_name=map_name, map_size=map_size, plot=plot, seed=seed)
        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            map_dims=map_dims,
            plot=plot,
            seed=seed,
            cell_size=cell_size,
        )
        path, simplified_path = tp2.planner.run(
            start=start, # in meters
            goal=goal, # in meters
            use_Astar=False,
            use_8_connected=False,
        )
        print(f"Path: {path}")
        print(f"Simplified Path: {simplified_path}")

        input("Press enter to run Roadmap (Grid) Original Path...")

        # Original Path
        tp2.start_controller(
            waypoints=path,
            gain_x=0.5,
            gain_y=0.5,
            gain_theta=0.1,
        )

        input("Press enter to run Roadmap (Grid) Simplified Path...")

        # Simplified Path
        tp2.start_controller(
            waypoints=simplified_path,
            gain_x=0.5,
            gain_y=0.5,
            gain_theta=0.1,
        )


        # 2.2) Potential Fields
        planner_type = "potential_fields"
        input("Press enter to setup Potential Fields...")
        
        goal_pos = (3.5, 3.5, 165) # x, y, theta

        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            plot=plot,
            goal_pos=goal_pos,
        )

        # This would be to plot
        # path = tp2.planner.run(
        #     start=(-3.5, -3.5), # in meters
        #     goal=(3.5, 3.5), # in meters,
        #     step_size=100.0, # in pixels
        #     max_iters=3000,
        #     goal_sample_rate=0.1, # percentage
        #     animate=True,
        #     collision_resolution=1, # in pixels
        # )

        tp2.start_controller()

        # 2.3) RRT
        input("Press enter to start planning with RRT...")
        planner_type = "rrt"
        tp2.setup_planner(
            planner_type=planner_type,
            map_name=map_name,
            map_size=map_size,
            plot=plot,
            seed=seed,
            map_dims=map_dims,
        )
        tree, edges, path_px, path = tp2.planner.run(
            start=start, # in meters
            goal=goal, # in meters,
            step_size=100.0, # in pixels
            max_iters=3000,
            goal_sample_rate=0.1, # percentage
            animate=True,
            collision_resolution=1, # in pixels
        )
        
        input("Press enter to run RRT...")
        
        print(f"Path: {path}")
        tp2.start_controller(
            waypoints=path,
            gain_x=0.5,
            gain_y=0.5,
            gain_theta=0.1,
        )

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        exit(-1)