import os
import time

print("Waiting for CoppeliaSim to start...")
time.sleep(3)

print("Testing imports...")
import roboticstoolbox as rtb  # noqa: F401, E402
from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # noqa: F401, E402
from swift import Swift  # noqa: F401, E402

from CoppeliaSimAPI import CoppeliaSimAPI  # noqa: F401, E402

print("Python stack OK")

port = os.environ.get("COPPELIA_ZMQ_PORT", 23000)

try:
    sim = CoppeliaSimAPI(port=port, stepping=False, verbose=0)
except Exception as e:
    print("CoppeliaSim not reachable (this is fine in CI):", e)

print("Starting simulation thread...")
sim.start()

print("Done.")
