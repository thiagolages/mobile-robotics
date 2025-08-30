import os
import time
from abc import abstractmethod

from CoppeliaSimAPI import CoppeliaSimAPI  # noqa: F401, E402


class Assignment:
    def __init__(self, sleep=True, auto_start=True, verbose=0):
        if sleep:
            print("Waiting for CoppeliaSim to start...")
            time.sleep(3)
        self.port = os.environ.get("COPPELIA_ZMQ_PORT", 23000)
        try:
            self.simAPI = CoppeliaSimAPI(
                port=self.port, stepping=False, verbose=verbose
            )
            self.sim = self.simAPI.sim
            if auto_start:
                self.simAPI.start()
        except Exception as e:
            print("CoppeliaSim not reachable (this is fine in CI):", e)
            exit(1)

    @abstractmethod
    def run(self):
        pass
