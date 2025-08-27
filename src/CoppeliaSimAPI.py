from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaSimAPI:
    def __init__(self, port=23000, stepping=True, verbose=None):
        """
        Initialize the CoppeliaSim API client.
        """
        # Create a RemoteAPIClient instance to connect to CoppeliaSim
        # The port number should match the one set in CoppeliaSim's remote API settings
        # port as set via -GzmqRemoteApi.rpcPort
        print("Trying to connect to CoppeliaSim API on port", port)
        self.client = RemoteAPIClient("localhost", port, verbose=verbose)
        self.sim = self.client.require("sim")

        print("Connected to CoppeliaSim API.")
        print("API version:", self.sim.getInt32Param(self.sim.intparam_program_version))

        self.sim.setStepping(stepping)
        self.dt = self.sim.getSimulationTimeStep()
        print("Simulation time step:", self.dt)

    def start(self):
        self.sim.startSimulation()

    def stop(self):
        self.sim.stopSimulation()

    def step(self):
        self.sim.step()

    def get_simulation_time(self):
        return self.sim.getSimulationTime()
