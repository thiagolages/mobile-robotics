# hexapod_controller.py
import math
from typing import (
    List,
    Optional,
    Sequence,
)


class HexapodController:
    """
    Python port of the original Lua child script from /hexapod.
    Use with the CoppeliaSim ZeroMQ Remote API from an external process.

    Typical usage:
        client = RemoteAPIClient()
        sim = client.getObject('sim')
        hexapod = HexapodController(sim, client=client)
        hexapod.init()  # find objects, build IK, set defaults

        # in your sim loop:
        while running:
            dt = sim.getSimulationTimeStep()
            hexapod.update(dt)
            sim.step()  # or your own stepping policy
    """

    def __init__(self, sim, simIK=None, client=None):
        """
        Args:
            sim:     remote API 'sim' object (client.getObject('sim')).
            simIK:   remote API 'simIK' object (optional). If None, we’ll try client.getObject('simIK').
            client:  RemoteAPIClient instance (optional, used to fetch simIK if not provided).
        """
        self.sim = sim
        if simIK is not None:
            self.simIK = simIK
        else:
            if client is None:
                raise ValueError(
                    "Provide either simIK or client so we can fetch simIK."
                )
            self.simIK = client.getObject("simIK")

        # Handles / cached data:
        self.antBase = None
        self.legBase = None
        self.simLegTips: List[int] = []
        self.simLegTargets: List[int] = []
        self.initialPos: List[List[float]] = (
            []
        )  # tip positions rel. to legBase
        self.legMovementIndex = [1, 4, 2, 6, 3, 5]  # same sequence as Lua

        # IK:
        self.ikEnv = None
        self.ikGroup = None

        # Motion state:
        self.stepProgression = 0.0
        self.realMovementStrength = 0.0

        # Step parameters (Lua's movData):
        self.movData = {
            "vel": 0.5,
            "amplitude": 0.16,
            "height": 0.04,
            "dir": 0.0,  # radians
            "rot": 0.0,  # rotation mode factor (-1..1 typically)
            "strength": 0.0,  # 0..1 smoothing to realMovementStrength
        }

        # Body pose motion helpers (from sysCall_thread):
        self.sizeFactor = 1.0
        self.vel = 0.05
        self.accel = 0.05
        self.initialP = [0.0, 0.0, 0.0]
        self.initialO = [0.0, 0.0, 0.0]

    # -------------------------
    # Initialization (ported from sysCall_init)
    # -------------------------
    def init(self):
        # Object paths are absolute since the original child script is removed:
        self.antBase = self.sim.getObject("/hexapod/base")
        self.legBase = self.sim.getObject("/hexapod/legBase")

        self.simLegTips = [
            self.sim.getObject(f"/hexapod/footTip{i}") for i in range(6)
        ]
        self.simLegTargets = [
            self.sim.getObject(f"/hexapod/footTarget{i}") for i in range(6)
        ]

        # Initial tip positions relative to legBase:
        self.initialPos = [
            self.sim.getObjectPosition(self.simLegTips[i], self.legBase)
            for i in range(6)
        ]

        self.stepProgression = 0.0
        self.realMovementStrength = 0.0

        # IK environment/group, equivalent to simIK.addElementFromScene in Lua:
        simBase = self.sim.getObject(
            "/hexapod"
        )  # base scene object for extraction
        self.ikEnv = self.simIK.createEnvironment()
        self.ikGroup = self.simIK.createGroup(self.ikEnv)

        for i in range(len(self.simLegTips)):
            self.simIK.addElementFromScene(
                self.ikEnv,
                self.ikGroup,
                simBase,
                self.simLegTips[i],
                self.simLegTargets[i],
                self.simIK.constraint_position,
            )

        # Default motion/pose setup (equivalent to sysCall_thread beginning):
        self.sizeFactor = self.sim.getObjectSizeFactor(self.antBase)
        self.vel = 0.05
        self.accel = 0.05

        # Small initial body lowering:
        self.initialP = [0.0, 0.0, 0.0]
        self.initialO = [0.0, 0.0, 0.0]
        p0 = self.initialP.copy()
        p0[2] -= 0.03 * self.sizeFactor
        self.move_to_pose(
            self.legBase, self.antBase, p0, self.initialO, self.vel, self.accel
        )

    # -------------------------
    # Per-step update (ported from sysCall_actuation)
    # -------------------------
    def update(self, dt: Optional[float] = None):
        """
        Call this each simulation step (dt in seconds). If dt is None, we read it from the sim.
        """
        if dt is None:
            dt = self.sim.getSimulationTimeStep()

        # Smooth movement strength towards target
        dx = self.movData["strength"] - self.realMovementStrength
        if abs(dx) > dt * 0.1:
            dx = (abs(dx) * dt * 0.5) / dx  # preserves sign, matches Lua trick
        self.realMovementStrength += dx

        # For each leg, compute the offset along the cyclic gait and set target pos
        for leg in range(6):
            # Lua: sp=(stepProgression+(legMovementIndex[leg]-1)/6) % 1  with 1-based indexing
            sp = (
                self.stepProgression + (self.legMovementIndex[leg] - 1) / 6.0
            ) % 1.0

            offset = [0.0, 0.0, 0.0]
            if sp < (1.0 / 3.0):
                offset[0] = sp * 3.0 * self.movData["amplitude"] / 2.0
            else:
                if sp < (1.0 / 3.0 + 1.0 / 6.0):
                    s = sp - 1.0 / 3.0
                    offset[0] = (
                        self.movData["amplitude"] / 2.0
                        - self.movData["amplitude"] * s * 6.0 / 2.0
                    )
                    offset[2] = s * 6.0 * self.movData["height"]
                else:
                    if sp < (2.0 / 3.0):
                        s = sp - 1.0 / 3.0 - 1.0 / 6.0
                        offset[0] = -self.movData["amplitude"] * s * 6.0 / 2.0
                        offset[2] = (1.0 - s * 6.0) * self.movData["height"]
                    else:
                        s = sp - 2.0 / 3.0
                        offset[0] = (
                            -self.movData["amplitude"] * (1.0 - s * 3.0) / 2.0
                        )

            # Direction & rotation coupling, identical math to Lua:
            ip = self.initialPos[leg]
            md = self.movData["dir"] + abs(self.movData["rot"]) * math.atan2(
                ip[0] * self.movData["rot"], -ip[1] * self.movData["rot"]
            )

            offset2 = [
                offset[0] * math.cos(md) * self.realMovementStrength,
                offset[0] * math.sin(md) * self.realMovementStrength,
                offset[2] * self.realMovementStrength,
            ]
            p = [ip[0] + offset2[0], ip[1] + offset2[1], ip[2] + offset2[2]]

            # Set the desired foot target (IK resolves joint movements afterward):
            self.sim.setObjectPosition(
                self.simLegTargets[leg], p, self.legBase
            )

        # Run IK (syncWorlds=true, allowError=true)
        self.simIK.handleGroup(
            self.ikEnv, self.ikGroup, {"syncWorlds": True, "allowError": True}
        )

        # Advance gait progression
        self.stepProgression += dt * self.movData["vel"]

    # -------------------------
    # Step mode (ported setStepMode)
    # -------------------------
    def set_step_mode(
        self,
        stepVelocity: float,
        stepAmplitude: float,
        stepHeight: float,
        movementDirection_deg: float,
        rotationMode: float,
        movementStrength: float,
    ):
        """
        Mirrors Lua's setStepMode(...):

        movementDirection_deg: direction in degrees (converted to radians internally)
        rotationMode: factor that modulates rotational coupling (-1..1 typical)
        movementStrength: 0..1 (smoothed in update)
        """
        self.movData = {
            "vel": stepVelocity,
            "amplitude": stepAmplitude,
            "height": stepHeight,
            "dir": math.pi * movementDirection_deg / 180.0,
            "rot": rotationMode,
            "strength": movementStrength,
        }

    # -------------------------
    # Pose motion utilities (ported moveToPose & moveBody)
    # -------------------------
    def move_to_pose(
        self,
        obj: int,
        relObj: int,
        pos: Sequence[float],
        euler: Sequence[float],
        vel: float,
        accel: float,
    ):
        """
        Thin wrapper for sim.moveToPose with same parameters/metric as in Lua.
        """
        params = {
            "object": obj,
            "relObject": relObj,
            "targetPose": self.sim.buildPose(list(pos), list(euler)),
            "maxVel": [vel],
            "maxAccel": [accel],
            "maxJerk": [0.1],
            "metric": [1, 1, 1, 0.1],
        }
        self.sim.moveToPose(params)

    def move_body(self, index: int):
        """
        Port of moveBody(index) from Lua. Applies a short body motion pattern.
        """
        p = self.initialP.copy()
        o = self.initialO.copy()

        # Always start by moving to nominal pose:
        self.move_to_pose(
            self.legBase, self.antBase, p, o, self.vel, self.accel
        )

        if index == 0:
            # up/down
            p[2] = p[2] - 0.03 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel * 2.0, self.accel
            )
            p[2] = p[2] + 0.03 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel * 2.0, self.accel
            )

        elif index == 1:
            # 4x twisting
            o[0] += 5 * math.pi / 180.0
            o[1] -= 5 * math.pi / 180.0
            o[2] -= 15 * math.pi / 180.0
            p[0] -= 0.03 * self.sizeFactor
            p[1] += 0.015 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

            o[0] -= 10 * math.pi / 180.0
            o[2] += 30 * math.pi / 180.0
            p[1] -= 0.04 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

            o[0] += 10 * math.pi / 180.0
            o[1] += 10 * math.pi / 180.0
            p[1] += 0.03 * self.sizeFactor
            p[0] += 0.06 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

            o[0] -= 10 * math.pi / 180.0
            o[2] -= 30 * math.pi / 180.0
            p[1] -= 0.03 * self.sizeFactor
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

        elif index == 2:
            # rolling
            o[0] += 17 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[0] -= 34 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[0] += 17 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

        elif index == 3:
            # pitching
            o[1] += 15 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[1] -= 30 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[1] += 15 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

        elif index == 4:
            # yawing
            o[2] += 30 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[2] -= 60 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )
            o[2] += 30 * math.pi / 180.0
            self.move_to_pose(
                self.legBase, self.antBase, p, o, self.vel, self.accel
            )

    # -------------------------
    # Optional: demo patterns (ported comments from sysCall_thread)
    # -------------------------
    def demo_on_the_spot(
        self, walkingVel: float, maxStep: float, stepHeight: float
    ):
        self.set_step_mode(walkingVel, maxStep, stepHeight, 0.0, 0.0, 0.0)
        self.move_body(0)
        self.move_body(1)
        self.move_body(2)
        self.move_body(3)
        self.move_body(4)

    def demo_forward_fixed_body(
        self,
        walkingVel: float,
        maxStep: float,
        stepHeight: float,
        seconds: float,
    ):
        self.set_step_mode(walkingVel, maxStep, stepHeight, 0.0, 0.0, 1.0)
        # You can run a timed loop in your driver: repeatedly call update(dt) for 'seconds'
        # Then stop:
        # self.set_step_mode(walkingVel, maxStep, stepHeight, 270.0, 0.0, 0.0)

    def demo_rotate_on_spot(
        self,
        walkingVel: float,
        maxStep: float,
        stepHeight: float,
        seconds: float,
    ):
        self.set_step_mode(
            walkingVel, maxStep * 0.5, stepHeight, 0.0, 1.0, 1.0
        )
        # Run your loop for 'seconds', then:
        # self.set_step_mode(walkingVel, maxStep * 0.5, stepHeight, 0.0, 0.0, 0.0)
