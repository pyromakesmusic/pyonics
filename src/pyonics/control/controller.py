"""
DEVELOPER COMMENTS

Keep at bottom level, so no references to other user-created libraries. Low level control algorithms. Probably best
to optimize with C/C++.
"""

"""
STANDARD LIBRARY IMPORTS
"""
import pandas as pd
import numpy as np
import argparse
import asyncio
import math

"""
OUTSIDE LIBRARY IMPORTS
"""
import klampt
import klampt.sim
import klampt.math.vectorops as kmv
import klampt.math.so3 as so3
import klampt.model.contact as kmc
import klampt.plan.robotplanning as kmrp
import klampt.plan.cspace as kmcs


"""
CUSTOM LIBRARY IMPORTS
"""

from .messages import AsyncServer, AsyncTestClient

"""
CLASS DEFINITIONS
"""
"""
Hardware
"""
class MuscleEmulator(klampt.sim.ActuatorEmulator):
    """
    row: A single dataframe row with muscle information.
    controller: A robot controller.

    Refers to exactly one McKibben muscle, with all associated attributes.
    This may end up being an interface for both an Actuator and a simulated ActuatorEmulator, running simultaneously.
    """
    def __init__(self, row, controller, sim):
        """
        Takes a dataframe row containing muscle information, a world model, a simulator, and a controller.
        """
        klampt.sim.ActuatorEmulator.__init__(self)
        self.controller = controller
        self.sim = sim
        self.a = int(row["link_a"])  # Gets index of the row of link a
        self.b = int(row["link_b"])

        """
        The below values describe the displacement of the muscle attachment from the origin of the robot link.
        """
        self.delta_a = [float(s) for s in row["transform_a"].split(",")]
        self.delta_b = [float(s) for s in row["transform_b"].split(",")]

        # This starts out fine, but may eventually need to be updated each time step according to link position

        # Now we add some attributes that the simulated and real robot will share
        self.geometry = klampt.GeometricPrimitive()
        world_a, world_b = self.geometryCalc()
        self.geometry.setSegment(world_a, world_b)


        self.turns = row["turns"]  # Number of turns in the muscle fiber
        self.weave_length = row["weave_length"]  # weave length - should be shorter than l_0
        self.max_pressure = row["max_pressure"]  # want this to autoscale for now, eventually static
        self.r_0 = row["r_0"]  # resting radius - at nominal relative pressure
        self.l_0 = row["l_0"]  # resting length - at nominal relative pressure
        self.length = self.l_0  # For calculation convenience. self.length should change eache time step
        self.displacement = 0  # This is a calculated value; should initialize at 0
        self.pressure = 0  # Should be pressure relative to external, so initialize at 0 - need units eventually
        self._k = (self.weave_length ** 2) / (4 * math.pi * self.turns ** 2) # Constant expression for calculations

    def collides(self):
        """
        Klampt syntactical sugar so this returns as having collision properties.
        """
        return True

    def withinDistance(self):
        """
        Same as above for shell.
        """
        return True

    def distance(self):
        """
        Same as above, returns a numeric value that is the collision distance.
        """
        return 0.8
    def geometryCalc(self):
        # 1. get link bodies
        body_a = self.sim.body(self.controller.robot.link(self.a))
        body_b = self.sim.body(self.controller.robot.link(self.b))

        # 2. compute world attachment points (WITH ROTATION)
        R_a, t_a = body_a.getTransform()
        R_b, t_b = body_b.getTransform()

        world_a = kmv.add(so3.apply(R_a, self.delta_a), t_a)
        world_b = kmv.add(so3.apply(R_b, self.delta_b), t_b)
        return world_a, world_b

    def process(self, commands, dt):
        if commands and 'pressure' in commands:
            self.pressure = commands['pressure']

    def substep(self, dt):
        world_a, world_b = self.geometryCalc()
        self.geometry.setSegment(world_a, world_b)

        direction = kmv.sub(world_a, world_b)
        length = kmv.norm(direction)

        if length < 1e-6:
            return

        unit = kmv.div(direction, length)
        displacement = length - self.l_0

        self.length = length
        self.displacement = displacement

        # 4. compute force magnitude
        force_mag = (self.pressure * self._k) * \
                    (((self.weave_length) / math.sqrt(3) + displacement) ** 2 - 1)

        force_vec = kmv.mul(unit, force_mag)

        # 1. get link bodies
        body_a = self.sim.body(self.controller.robot.link(self.a))
        body_b = self.sim.body(self.controller.robot.link(self.b))
        # 5. apply equal and opposite forces
        body_a.applyForceAtWorldPoint(kmv.mul(force_vec, -1), world_a)
        body_b.applyForceAtWorldPoint(force_vec, world_b)

    def pressure_autoscale(self):
        if self.pressure > self.max_pressure:  # autoscaling algorithm
            self.max_pressure = self.pressure

    def appearance(self):
        app = klampt.Appearance()
        app.setDraw(2, True)
        app.setColor(0, 1, 0, 1)
        return app

class MuscleGroup():
    pass

"""
Robot Controller
"""
class ExoController(klampt.control.OmniRobotInterface):
    """
    Most low level hardware controller. No display, but contains enough of a world to start generating HUD elements.
    """

    # Initialization
    def __init__(self, config_data):
        """
        Initializes the controller. Should work on a physical or simulated robot equivalently or simultaneously.
        """
        # print(config_data)
        self.config = config_data
        self.server = None
        self.collider = None

        if config_data["has_robworld"]:
            self.world = klampt.io.load('WorldModel', config_data["world_path"])  # Loads the world, this is where it's made
            self.world.loadRobot(config_data["core"])  # Loads the robot geometry
            self.robot = self.world.robot(0)
            self.interface = klampt.control.OmniRobotInterface.__init__(self, self.robot)

        self.dt = config_data["timestep"]  # Sets the core robot clock
        # Setting initial muscle pressure to zero
        self.pressures = None

    """
    Kinematics and Control
    """

    def set_pressures(self, address, *osc_args):
        """
        address: OSC address string (e.g. '/pressures')
        osc_args: pressure values
        """
        print("[SET_PRESSURES]", address, osc_args)
        self.pressures = list(osc_args)

    def controlRate(self):
        return self.dt

    async def setup_osc_server(self):
        # Does the mapping and last minute settings stuff necessary to begin controller idle
        # Changed name from idle_configuration
        self.server = AsyncServer(self.config["address"], self.config["port"], "/pressures", self.set_pressures)
        await self.server.map("/pressures", self.set_pressures)
        await self.server.make_endpoint()

    """
    DIAGNOSTIC
    """

    async def enable_osc_logging(self, enabled: bool = True):
        if self.server:
            self.server.enable_osc_logging(enabled)