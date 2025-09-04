from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaSimAPI:
    def __init__(self, port=23000, stepping=True, verbose=None):
        """
        Initialize the CoppeliaSim API client.
        """
        # Create a RemoteAPIClient instance to connect to CoppeliaSim
        # The port number should match the one in CoppeliaSim's remote API
        # settings port as set via -GzmqRemoteApi.rpcPort
        print("-" * 40)
        print(f"Trying to connect to CoppeliaSim API on port {port}")
        self.client = RemoteAPIClient("localhost", port, verbose=verbose)
        self.sim = self.client.require("sim")
        self.simIK = self.client.require("simIK")

        print("Connected to CoppeliaSim API.")
        api_version = self.sim.getInt32Param(self.sim.intparam_program_version)
        print(f"API version: {api_version}")
        self.stepping = stepping
        self.sim.setStepping(self.stepping)
        self.dt = self.sim.getSimulationTimeStep()
        print("Simulation time step:", self.dt)
        print("-" * 40)

    def start(self):
        self.sim.startSimulation()

    def resume(self):
        self.start()

    def pause(self):
        self.sim.pauseSimulation()

    def stop(self):
        self.sim.stopSimulation()

    def step(self):
        self.sim.step()

    def get_simulation_time(self):
        return self.sim.getSimulationTime()
