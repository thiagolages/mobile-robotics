import os
import time

from CoppeliaSimAPI import CoppeliaSimAPI

print("Waiting for CoppeliaSim to start...")
time.sleep(3)

print("Imports...")
import roboticstoolbox as rtb  # noqa: F401
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from swift import Swift  # noqa: F401

print("Python stack OK")

port = os.environ.get("COPPELIA_ZMQ_PORT", 23000)

try:
    sim = CoppeliaSimAPI(port=port, stepping=False, verbose=0)
except Exception as e:
    print("CoppeliaSim not reachable (this is fine in CI):", e)

print("Starting simulation thread...")
sim.start()

print("Done.")
