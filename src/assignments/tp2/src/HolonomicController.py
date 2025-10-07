import time
from Controller import Controller
import numpy as np

def Rz(theta):
  
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                      [ np.sin(theta), np.cos(theta) , 0 ],
                      [ 0            , 0             , 1 ]])
                      
class HolonomicController(Controller):
    def __init__(self, sim, robot_name, planner, gain_x=0.1, gain_y=0.1, gain_theta=0.1):
        super().__init__(sim, robot_name)
        self.sim = sim
        self.robot_name = robot_name
        self.planner = planner
        self.gain_x = gain_x
        self.gain_y = gain_y
        self.gain_theta = gain_theta
    
        # Handle para as juntas das RODAS
        self.wheel1 = self.sim.getObject('/' + self.robot_name + '/wheel0_joint')
        self.wheel2 = self.sim.getObject('/' + self.robot_name + '/wheel1_joint')
        self.wheel3 = self.sim.getObject('/' + self.robot_name + '/wheel2_joint')
                    
        # Dados Robotino
        L = 0.135   # Metros
        r = 0.040   # Metros  
        
        # Cinemática Direta
        self.Mdir = np.array([[-r/np.sqrt(3), 0, r/np.sqrt(3)], [r/3, (-2*r)/3, r/3], [r/(3*L), r/(3*L), r/(3*L)]])
        
        # Waypoint following parameters
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 0.2  # meters
        self.max_simulation_time = 900  # seconds

    def set_waypoints(self, waypoints):
        """Set the waypoints for the robot to follow.
        
        Args:
            waypoints: List of waypoints, each waypoint is [x, y, theta] or [x, y]
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        print(f"Set {len(waypoints)} waypoints: {waypoints}")

    def run(self, waypoints=None, gain_x=None, gain_y=None, gain_theta=None):
        """Run the controller to follow waypoints.
        
        Args:
            waypoints: List of waypoints to follow. If None, uses previously set waypoints.
                      Each waypoint should be [x, y, theta] or [x, y] (theta defaults to 0)
            gain_x: Gain for the x-axis. If None, uses previously set gain.
            gain_y: Gain for the y-axis. If None, uses previously set gain.
            gain_theta: Gain for the theta-axis. If None, uses previously set gain.
        """
        if gain_x is None:
            gain_x = self.gain_x
        if gain_y is None:
            gain_y = self.gain_y
        if gain_theta is None:
            gain_theta = self.gain_theta
        
        if waypoints is not None:
            self.set_waypoints(waypoints)
        
        if not self.waypoints:
            print("No waypoints set. Please provide waypoints to the run() method or call set_waypoints() first.")
            return
        
        try:
            sim = self.sim # shortcut
            robotHandle = sim.getObject('/' + self.robot_name)
            
            # Inicia a simulação
            # Wait a few seconds to start the simulation
            # print("Waiting 3 seconds to start the simulation...")
            # time.sleep(3)
            sim.startSimulation()
            sim.step()

            gain = np.array([[gain_x, 0, 0], [0, gain_y, 0], [0, 0, gain_theta]])
            
            print("Starting robot control loop...")
            print(f"Following {len(self.waypoints)} waypoints")
            count = 0
            while (sim_time := sim.getSimulationTime()) <= self.max_simulation_time:
                count += 1
                # print(f"Simulation time: {sim_time:.2f} [s]")
                
                # Check if we've reached all waypoints
                if self.current_waypoint_index >= len(self.waypoints):
                    print("All waypoints reached!")
                    break
                
                # Get current waypoint
                current_waypoint = self.waypoints[self.current_waypoint_index]
                
                # Ensure waypoint has 3 elements [x, y, theta]
                if len(current_waypoint) == 2:
                    qgoal = np.array([current_waypoint[0], current_waypoint[1], 0.0])
                else:
                    qgoal = np.array(current_waypoint)
                
                # Get current robot position
                pos = sim.getObjectPosition(robotHandle, sim.handle_world)
                ori = sim.getObjectOrientation(robotHandle, sim.handle_world)
                q = np.array([pos[0], pos[1], ori[2]])

                # print(f"q: {q}, qgoal: {qgoal}")
                
                # Calculate error
                error = qgoal - q
                position_error = np.linalg.norm(error[:2])
                # if count % 10 == 0:
                #     print(f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} = {current_waypoint}, Error: {position_error:.3f}")
                
                # Check if current waypoint is reached
                if position_error < self.waypoint_tolerance:
                    print(f"Reached waypoint {self.current_waypoint_index + 1} = {current_waypoint}")
                    self.current_waypoint_index += 1
                    continue

                # Controller
                qdot = gain @ error
                
                # Cinemática Inversa
                # w1, w2, w3
                Minv = np.linalg.inv(Rz(q[2]) @ self.Mdir)
                u = Minv @ qdot
                
                # Enviando velocidades
                sim.setJointTargetVelocity(self.wheel1, u[0])
                sim.setJointTargetVelocity(self.wheel2, u[1])
                sim.setJointTargetVelocity(self.wheel3, u[2]) 
                        
                sim.step()

            # Parando o robô
            print("Stopping robot...")
            sim.setJointTargetVelocity(self.wheel1, 0)
            sim.setJointTargetVelocity(self.wheel2, 0)
            sim.setJointTargetVelocity(self.wheel3, 0)
            
        except Exception as e:
            print(f"An error occurred: {e}")
            
        # Parando a simulação
        sim.stopSimulation()

        print('Program ended')