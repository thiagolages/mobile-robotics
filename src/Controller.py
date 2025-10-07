# from abc import ABC, abstractmethod

# class Controller(ABC):
#     def __init__(self, sim, robot_name, objects):
#         self.sim = sim
#         self.robot_name = robot_name
#         self.objects = objects

#     @abstractmethod
#     def setup(self):
#         pass

#     @abstractmethod
#     def run(self):
#         pass

# class HolonomicController(Controller):
#     def __init__(self, sim, robot_name, objects):
#         super().__init__(sim, robot_name, objects)

#     def setup(self):
#         pass

#     def run(self):
        
#         try:
#             # Handle para o ROBÔ
#             robotname = self.robot_name
#             sim = self.sim
#             robotHandle = sim.getObject('/' + robotname)
            
#             # Handle para as juntas das RODAS
#             wheel1 = sim.getObject('/' + robotname + '/wheel0_joint')
#             wheel2 = sim.getObject('/' + robotname + '/wheel1_joint')
#             wheel3 = sim.getObject('/' + robotname + '/wheel2_joint')
                    
#             # Dados Robotino
#             L = 0.135   # Metros
#             r = 0.040   # Metros  
            
#             # Cinemática Direta
#             Mdir = np.array([[-r/np.sqrt(3), 0, r/np.sqrt(3)], [r/3, (-2*r)/3, r/3], [r/(3*L), r/(3*L), r/(3*L)]])
                
#             # Goal configuration (x, y, theta)    
#             qgoal = np.array([3, 3, np.deg2rad(90)])
#             #qgoal = np.array([3, -3, np.deg2rad(-90)])
#             #qgoal = np.array([0, 0, np.deg2rad(0)])
            
#             # Lembrar de habilitar o 'Real-time mode'    
#             # Parar a simulação se estiver executando
#             initial_sim_state = sim.getSimulationState()
#             if initial_sim_state != 0:
#                 sim.stopSimulation()
#                 time.sleep(1)
            
#             # Inicia a simulação
#             sim.startSimulation()
#             sim.step()

#             # Frame que representa o Goal
#             goalFrame = sim.getObject('/Goal')
#             sim.setObjectPosition(goalFrame, [qgoal[0], qgoal[1], 0], sim.handle_world)
#             sim.setObjectOrientation(goalFrame, [0, 0, qgoal[2]], sim.handle_world)
            
#             gain = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
#             #gain = np.array([[0.3, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
            
#             print("Starting robot control loop...")
#             while (sim_time := sim.getSimulationTime()) <= 90:
#                 print(f"Simulation time: {sim_time:.2f} [s]")
                
#                 dt = sim.getSimulationTimeStep()
                        
#                 pos = sim.getObjectPosition(robotHandle, sim.handle_world)
#                 ori = sim.getObjectOrientation(robotHandle, sim.handle_world)

#                 q = np.array([pos[0], pos[1], ori[2]])
                
#                 error = qgoal - q
#                 print(f"Error: {np.linalg.norm(error[:2])}")
                
#                 # Margem aceitável de distância
#                 if (np.linalg.norm(error[:2]) < 0.05):
#                     break

#                 # Controller
#                 qdot = gain @ error
                
#                 # Cinemática Inversa
#                 # w1, w2, w3
#                 Minv = np.linalg.inv(Rz(q[2]) @ Mdir)
#                 u = Minv @ qdot
                
#                 # Enviando velocidades
#                 sim.setJointTargetVelocity(wheel1, u[0])
#                 sim.setJointTargetVelocity(wheel2, u[1])
#                 sim.setJointTargetVelocity(wheel3, u[2]) 
                        
#                 sim.step()

#             # Parando o robô
#             print("Stopping robot...")
#             sim.setJointTargetVelocity(wheel1, 0)
#             sim.setJointTargetVelocity(wheel2, 0)
#             sim.setJointTargetVelocity(wheel3, 0)
            
#         except Exception as e:
#             print(f"An error occurred: {e}")
            
#         # Parando a simulação
#         sim.stopSimulation()

#         print('Program ended')

# class DifferentialController(Controller):
#     def __init__(self, sim, robot_name, objects):
#         super().__init__(sim, robot_name, objects)

#     def setup(self):
#         pass

#     def run(self):
#         try:
#             # Connect to the CoppeliaSim server
#             client = RemoteAPIClient()
#             sim = client.require("sim")
#             sim.setStepping(True)

#             # Handle para o ROBÔ
#             robotname = 'Pioneer_p3dx'
#             robotHandle = sim.getObject('/' + robotname)
            
#             # Handle para as juntas das RODAS
#             l_wheel = sim.getObject('/' + robotname + '/'+ robotname + '_leftMotor')
#             r_wheel = sim.getObject('/' + robotname + '/'+ robotname + '_rightMotor')
            
#             # Dados Pioneer
#             # https://www.generationrobots.com/media/Pioneer3DX-P3DX-RevA.pdf
#             # L = 0.381   # Metros
#             # r = 0.0975  # Metros
            
#             L = 0.331
#             r = 0.09751
#             maxv = 1.0
#             maxw = np.deg2rad(45)
                
#             # Goal position (x, y)
#             pgoal = np.array([2.0, -2.0])
#             # pgoal = np.array([0.0, 0.0])
            
#             # Lembrar de habilitar o 'Real-time mode'    
#             # Parar a simulação se estiver executando
#             initial_sim_state = sim.getSimulationState()
#             if initial_sim_state != 0:
#                 sim.stopSimulation()
#                 time.sleep(1)
            
#             # Inicia a simulação
#             sim.startSimulation()
#             sim.step()

#             # Frame que representa o Goal
#             goalFrame = sim.getObject('/Goal')
#             sim.setObjectPosition(goalFrame, [pgoal[0], pgoal[1], 0], sim.handle_world)    
            
#             print("Starting robot control loop...")
#             while (sim_time := sim.getSimulationTime()) <= 90:
#                 # print(f"Simulation time: {sim_time:.2f} [s]")
                
#                 dt = sim.getSimulationTimeStep()
                        
#                 robotPos = sim.getObjectPosition(robotHandle, sim.handle_world)
#                 robotOri = sim.getObjectOrientation(robotHandle, sim.handle_world)
#                 robotConfig = np.array([robotPos[0], robotPos[1], robotOri[2]])
                
#                 dx, dy = pgoal - robotConfig[:2]
                
#                 # Apenas para interromper o loop
#                 rho = np.sqrt(dx**2 + dy**2)
#                 # print(f"Error (rho): {rho}")
#                 if rho <= .05:
#                     break           
                    
#                 kr = 1
#                 kt = 2
                
#                 v = kr*(dx*np.cos(robotConfig[2]) + dy*np.sin(robotConfig[2]))
#                 w = kt*(np.arctan2(dy,dx) - robotConfig[2])
                
#                 # Limit v,w to +/- max
#                 v = max(min(v, maxv), -maxv)
#                 w = max(min(w, maxw), -maxw)        
                
#                 wr = ((2.0*v) + (w*L))/(2.0*r)
#                 wl = ((2.0*v) - (w*L))/(2.0*r)
                
#                 # Enviando velocidades
#                 sim.setJointTargetVelocity(l_wheel, wl)
#                 sim.setJointTargetVelocity(r_wheel, wr)
                
#                 sim.step()

#             # Parando o robô
#             print("Stopping robot...")
#             sim.setJointTargetVelocity(l_wheel, 0)
#             sim.setJointTargetVelocity(r_wheel, 0)
            
#         except Exception as e:
#             print(f"An error occurred: {e}")
            
#         # Parando a simulação
#         sim.stopSimulation()

#         print('Program ended')
