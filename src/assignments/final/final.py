import os
import sys

# Get the absolute path to the src directory
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../"))
sys.path.append(src_path)

from CoppeliaSimAPI import CoppeliaSimAPI
from Assignment import Assignment
from utils import handle_sim_start, get_tf
import numpy as np
import time
import threading

class Final(Assignment):
    def __init__(self):
        super().__init__(sleep=False, auto_start=False)
        self._setup()
        self.sim_alive = True
        self.lock = threading.RLock()
        self.dt = self.simAPI.get_simulation_time_step()

    def _setup(self):
        self.excluded_objects = [
            "DefaultCamera",
            "XYZCameraProxy",
            "DefaultLights",
            "Floor",
        ]
        self.setup_handles()
        self.setup_drone_tfs()
        self.setup_target_tfs()

    def setup_handles(self):
        self.drone_handles = []
        self.target_handles = []
        for i in range(1, 6):
            self.drone_handles.append(self.sim.getObject(f"/drone_{i}"))
            self.target_handles.append(self.sim.getObject(f"/target_{i}_frame"))

    def setup_drone_tfs(self):
        self.drone_tfs = []
        for drone_handle in self.drone_handles:
            self.drone_tfs.append(get_tf(self.sim, drone_handle))
    
    def setup_target_tfs(self):
        self.target_tfs = []
        for target_handle in self.target_handles:
            self.target_tfs.append(get_tf(self.sim, target_handle))
    
    def loop(self):
        time.sleep(0.5) # wait some time to let the simulation start
        while True:
            with self.lock:
                if not self.sim_alive:
                    break
            self.sim.step()
            time.sleep(self.dt)

    def get_drone_next_pose(self, drone_id, time):
        drone_tf = self.drone_tfs[drone_id]
        target_tf = self.target_tfs[drone_id]
        next_pose = drone_tf @ target_tf
        return next_pose

    def run(self):
        handle_sim_start(self.sim)
        r = 1.5
        z = 2.0
        dt = 0.1
        
        # Wait for simulation to start
        self.sim.step()
        time.sleep(0.5) # wait some time to let the simulation start

        self.sim_state = self.sim.getSimulationState()
        while self.sim_state == self.sim.simulation_stopped or self.sim_state == self.sim.simulation_paused:
            print("Sim state = ", self.sim_state)
            time.sleep(0.1)
            self.sim_state = self.sim.getSimulationState()

        # Start main loop in a separate thread
        loop = threading.Thread(target=self.loop, daemon=True)
        loop.start()
        print("After loop start")
        start_time = time.time()
        for i in np.linspace(0, 10, int(10/0.2)):
            self.get_drone_next_pose(
                drone_id = i,
                time = i * dt,
            )
            theta = i * 2 * np.pi / 10
            pose = np.array([r*np.cos(theta), r*np.sin(theta), z, 0, 0, 0, 1])
            # print("Setting pose: ", pose)
            # self.sim.setObjectPose(self.drone_handle, pose)
            self.sim.setObjectPosition(self.target_handle, pose[:3])
            time.sleep(dt)
            with self.lock:
                self.sim_alive = False
    
        print("Total time taken: ", time.time() - start_time, " seconds")
        loop.join()
        

if __name__ == "__main__":
    final = Final()
    final.run()