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
    def __init__(self, row, controller):
        """
        Takes a dataframe row containing muscle information, a world model, a simulator, and a controller.
        """
        klampt.sim.ActuatorEmulator.__init__(self)
        self.controller = controller
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
        self.geometry.setSegment(self.transform_a, self.transform_b)


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

    def process(self, commands, dt):
        if commands and 'pressure' in commands:
            self.pressure = commands['pressure']

    def substep(self, dt):
        # 1. get link bodies
        body_a = self.sim.body(self.controller.robot.link(self.a))
        body_b = self.sim.body(self.controller.robot.link(self.b))

        # 2. compute world attachment points (WITH ROTATION)
        R_a, t_a = body_a.getTransform()
        R_b, t_b = body_b.getTransform()

        world_a = kmv.add(so3.apply(R_a, self.delta_a), t_a)
        world_b = kmv.add(so3.apply(R_b, self.delta_b), t_b)
        self.geometry.setSegment(world_a, world_b)

        # 3. compute length + direction
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

        # 5. apply equal and opposite forces
        body_a.applyForceAtWorldPoint(kmv.mul(force_vec, -1), world_a)
        body_b.applyForceAtWorldPoint(force_vec, world_b)

    def update_muscle_old(self, pressure):  # Should call every loop?
        """
        pressure: single float value. Starting at 0-1 but may make sense to put in terms of psi, bar or pascal.
        ================
        UPDATE 10.2.2023: A muscle is a spring with variable stiffness.

        Should apply two forces at points determined by self.transform_a and self.transform_b, moderated by the
        McKibben muscle formula.

        We want to calculate
        F: the force applied by the muscle.
        To do this we will need:
        p: relative pressure of the air chamber
        b: the muscle fiber weave length
        n: number of turns in the muscle fiber
        x: the displacement. This will probably take the most work to calculate.
        """
        # # MuscleEmulator transforms must update based on new link positions //// maybe not with applyForceAtLocalPoint()
        # self.link_a = self.controller.bones[self.a]
        # self.link_b = self.controller.bones[self.b]
        #
        # self.transform_a = kmv.add(self.link_a[1], self.delta_a)  # Adds link transform to muscle delta
        # self.transform_b = kmv.add(self.link_b[1], self.delta_b)

        self.geometry.setSegment(self.transform_a, self.transform_b)  # Should be updating the transform

        self.pressure = pressure  # Updates muscle pressure
        self.length = kmv.distance(self.transform_a, self.transform_b)
        self.displacement = self.length - self.l_0  # Calculates displacement based on new length

        # MuscleEmulator formula
        force = ((self.pressure * (self.weave_length)**2)/(4 * math.pi * (self.turns)**2)) * \
                (((self.weave_length)/math.sqrt(3) + self.displacement)**2 - 1)

        # Calculating a 3-tuple that gives a direction
        direction_a = kmv.sub(self.transform_a, self.transform_b)
        direction_b = kmv.mul(direction_a, -1) # Should just be the reverse of direction_a

        # Calculating unit vectors by dividing 3-tuple by its length
        unit_a = kmv.div(direction_a, self.length)
        unit_b = kmv.div(direction_b, self.length)  # Changed to division

        # Combining unit vectors and force magnitude to give a force vector
        force_a = kmv.mul(unit_a, force)  # Half (.5) because of Newton's Third Law,
        force_b = kmv.mul(unit_b, force)

        triplet_a = [self.b, force_a, self.transform_b]  # Should be integer, 3-tuple, transform
        triplet_b = [self.a, force_b, self.transform_a]  # Link to apply to, force vector to apply, transform at which to apply
        """
        These triplets are what is required to simulate the effect of the muscle contraction. Also, at some point I want
        to change the muscle color based on the pressure input.
        """
        return triplet_a, triplet_b

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
        # Loading all the muscles
        self.muscles = self.muscleLoader(config_data)
        # Setting initial muscle pressure to zero
        self.pressures = [0 for x in range(len(self.muscles))]

    def muscleLoader(self, config_df):
        """
        Given a dataframe with an ["attachments"] column containing a path
        to a .csv file detailing structured muscle parameters, generates a list of MuscleEmulator objects and
        assigns them to the robot model. This should generate all muscles.
        """
        with open(config_df["attachments"]) as attachments:
            muscleinfo_df = pd.read_csv(attachments, sep=";")  # This dataframe contains info on every muscle attachment
            rows = muscleinfo_df.shape[0]  # This is the number of rows, so the while loop should loop "row" many times

            muscle_objects = []  # Placeholder list, made to be empty and populated with all muscle objects.

            for x in range(rows):
                row = muscleinfo_df.iloc[x] # Locates the muscle information in the dataframe
                muscle = MuscleEmulator(row, self) # Calls the muscle class constructor, has robot controller as argument
                muscle_objects.append(muscle) # Adds the muscle to the list

            muscle_series = pd.Series(data=muscle_objects, name="muscle_objects")
            pressure_series = pd.Series(data=0, name="pressure")
            muscleinfo_df = pd.concat([muscleinfo_df, muscle_series, pressure_series], axis=1)

            """
            This dataframe should end with all the info in the muscle attachments CSV, plus corresponding muscle objects
            in each row.
            # """
            # print(str(muscleinfo_df) + " muscleinfo df") # Doing test prints
            return muscleinfo_df

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