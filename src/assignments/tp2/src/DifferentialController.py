from Controller import Controller
import numpy as np

class DifferentialController(Controller):
    def __init__(self, sim, robot_name, planner):
        super().__init__(sim, robot_name)
        self.sim = sim
        self.robot_name = robot_name
        self.planner = planner
        
        # Handle para o ROBÔ
        self.robotHandle = sim.getObject('/' + self.robot_name)
        
        # Handle para as juntas das RODAS
        self.l_wheel = sim.getObject('/' + self.robot_name + '/leftMotor')
        self.r_wheel = sim.getObject('/' + self.robot_name + '/rightMotor')
        
        # Dados Pioneer
        # https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf
        # L = 0.381   # Metros
        # r = 0.0975  # Metros
        
        self.L = 0.331
        self.r = 0.09751
        self.maxv = 1.0
        self.maxw = np.deg2rad(45)

        # Controller gains
        self.kv = 1
        self.kw = 1.5
        
        # Waypoint following parameters
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.1  # meters
        self.theta_tolerance = np.deg2rad(5)
        self.max_simulation_time = 900  # seconds

    def set_waypoints(self, waypoints):
        """Set the waypoints for the robot to follow.
        
        Args:
            waypoints: List of waypoints, each waypoint is [x, y] (theta is not used for differential drive)
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        print(f"Set {len(waypoints)} waypoints: {waypoints}")

    def run(self, goal_pos):
        """Run the controller to navigate to a goal position using De Luca and Oriolo equations.
        
        Args:
            goal_pos: Goal position as (x, y, theta) where theta is the desired orientation
        """
        if goal_pos is None or len(goal_pos) != 3:
            print("Invalid goal position. Please provide goal_pos as (x, y, theta).")
            return
        
        try:
            sim = self.sim # shortcut
            sim.step()

            print("Starting robot control loop...")
            print(f"Navigating to goal position: {goal_pos}")
            
            # Extract goal position and orientation
            goal_x, goal_y, goal_theta = goal_pos
            pgoal = np.array([goal_x, goal_y])
            
            while (sim_time := sim.getSimulationTime()) <= self.max_simulation_time:
                # Get current robot position
                robotPos = sim.getObjectPosition(self.robotHandle, sim.handle_world)
                robotOri = sim.getObjectOrientation(self.robotHandle, sim.handle_world)
                robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])
                
                x, y = robotConfig[:2]
                theta = robotConfig[2]

                # Ensure theta is within [-pi, pi]
                theta = np.arctan2(np.sin(theta), np.cos(theta))
                
                k_att = 1.0
                k_rep = 1.0
                # Calculate distance to goal
                vx_att, vy_att = k_att * (pgoal - robotConfig[:2])
                vx_rep, vy_rep = self.planner.get_repulsive_force(robotConfig[:2])

                # Scale attractive force
                vx_att, vy_att = 0.1 * vx_att, 0.1 * vy_att

                # Scale repulsive force
                vx_rep, vy_rep = 0.01 * vx_rep, 0.01 * vy_rep

                print(f"vx_att: {vx_att:.3f}, vy_att: {vy_att:.3f}, vx_rep: {vx_rep:.3f}, vy_rep: {vy_rep:.3f}")

                vx = vx_att + vx_rep
                vy = vy_att + vy_rep

                print(f"vx: {vx:.3f}, vy: {vy:.3f}")

                # distance to goal
                dx, dy  = pgoal[0] - x, pgoal[1] - y
                dist_to_goal = np.sqrt(dx**2 + dy**2)
                
                # Calculate orientation error
                theta_error = goal_theta - theta
                # Normalize angle to [-pi, pi]
                theta_error = np.arctan2(np.sin(theta_error), np.cos(theta_error))
                
                print(f"Distance to goal: {dist_to_goal:.3f}") #, Orientation error: {np.rad2deg(theta_error):.1f}°")
                
                # Check if goal is reached (both position and orientation)
                if dist_to_goal < self.waypoint_tolerance and abs(theta_error) < self.theta_tolerance:
                    print(f"Goal reached! Position: ({x:.3f}, {y:.3f}), Orientation: {np.rad2deg(theta):.1f}°")
                    break
                
                # Calculate control velocities using De Luca and Oriolo equations
                v = self.kv * (vx * np.cos(theta) + vy * np.sin(theta))
                w = self.kw * (np.arctan2(vy, vx) - theta)

                print(f"v: {v:.3f}, w: {w:.3f}")
                
                # Limit v,w to +/- max
                v = max(min(v, self.maxv), -self.maxv)
                w = max(min(w, self.maxw), -self.maxw)        
                
                # Wheel velocities
                wr = ((2.0*v) + (w*self.L))/(2.0*self.r)
                wl = ((2.0*v) - (w*self.L))/(2.0*self.r)
                
                # Sending velocities
                sim.setJointTargetVelocity(self.l_wheel, wl)
                sim.setJointTargetVelocity(self.r_wheel, wr)
                
                # Stepping
                sim.step()
            
            # Stop the robot
            print("Stopping robot...")
            sim.setJointTargetVelocity(self.l_wheel, 0)
            sim.setJointTargetVelocity(self.r_wheel, 0)
            
        except Exception as e:
            print(f"An error occurred: {e}")