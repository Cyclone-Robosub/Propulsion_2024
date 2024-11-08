import math
import numpy as np
from scipy.linalg import solve
import sympy


# Kinetics for RoboCyclone main control module

# /!\ Unless otherwise specified, all following units should be assumed as m, s, kg and products thereof  /!\
# /!\ For ease of use, any variables with non-standard units should have that unit in the variable name!  /!\
# /!\ eg: force_thruster is assumed to be newtons, force_thruster_kgf is clearly kgf                      /!\

# Definitions:
# velocity or acceleration (x,y,z) refers to (surge, sway, heave). Positive directions are bow, starboard, upward
# angular velocity or angular acceleration about (x,y,z) refers to (roll, pitch, yaw). Positive directions follow the
# right hand rule. (Thumb pointing bow, starboard, upward)
# Thrusters 0, 1, 2, 3 refer to vertical thrusters front-left, front-right, rear-left, rear-right
# Thrusters 4, 5, 6, 7 refer to horizontal thrusters front-left, front-right, rear-left, rear-right


# the purpose of this class is to solve for T, in the matrix formula F = dT + E + f
class ThrusterController:

    def __init__(self):
        # 3d position for center of mass, relative to an arbitrary origin position. Origin position should be static,
        # as all thruster positions will be relative to this origin. Mass center must not be the origin, as mass center
        # will change as modifications are made to the sub. Mass center can be determined experimentally, but I'm not
        # sure how to currently. Set to [0,0,0] as placeholder
        self.mass_center = [0, 0, 0]

        # 3D position for center of volume (center of upwards buoyancy force) center of displacement can be determined
        # experimentally by submerging the robot and recording its total displacement. Robot is then submerged to %50
        # displacement oriented on its x, y, and z axis to find total displacement. Set to [0,0,0] as placeholder
        # ideally, center of displacement should have the same (x,y) coordinates as center fo mass
        self.displacement_center = [0, 0, 0]
        rho_water = 997  # density of water
        force_gravity = -9.81  # force of gravity
        self.sub_mass = 10  # mass of sub, 10kg as placeholder
        self.sub_displacement = 0.1  # volume of total displacement, 0.1m^3 as placeholder
        self.force_weight = self.sub_mass * force_gravity
        self.force_buoyancy = self.sub_displacement * rho_water * force_gravity
        # we will use this center point to compute moments on the robot
        self.gravitational_center = [
            (self.force_weight * self.mass_center[i] + self.force_buoyancy * self.displacement_center[i]) /
            (self.force_buoyancy + self.force_weight) for i in range(3)]

        # velocity of current relative to robot. This value will be assumed to be zero until we have a good way to
        # estimate current using a feedback loop
        self.relative_current_flow = [0, 0, 0]

        self.position = [0, 0, 0]  # the robots current position in the pool, relative to its starting position
        self.velocity = [0, 0, 0]  # the robots current velocity
        self.acceleration = [0, 0, 0]  # the robots current acceleration

        # the robots current orientation, in radians, relative to its staring orientation
        self.angular_position = [0, 0, 0]
        self.angular_velocity = [0, 0, 0]
        self.angular_acceleration = [0, 0, 0]

        # 3-dimensional (x,y,z) positions of thrusters as they relate to an arbitrary (000)
        # set to [0,0,0] as placeholder
        self.thruster_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                                   [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # 3- dimension unit vectors representing the direction of each thruster
        # (x,y,z) correspond to (sway, surge, heave) in this script
        # /!\ double check thruster directions
        self.thruster_directions = [[0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0],
                                    [2 ** (-1 / 2), 2 ** (-1 / 2), 0.0], [2 ** (-1 / 2), -2 ** (-1 / 2), 0.0],
                                    [2 ** (-1 / 2), 2 ** (-1 / 2), 0.0], [2 ** (-1 / 2), -2 ** (-1 / 2), 0.0]]

        # Axial drag force of each thruster about the x, y and z axis in (N*m).
        # This represents the rotation force projected about the thrusters own axis due to drag.
        # These values may be negligible and may need to be determined experimentally.
        # Until determined further, these forces will be assumed to be negligible
        # Set to [0,0,0] as placeholder
        self.thruster_rotational_drag_force = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0],
                                               [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

        # this is the current voltage available to each thruster. This value should be coded as 10, 12, 14, 16,
        # 18 0r 20, as these are the values with provided tables from BlueRobotics. Set to 16 as placeholder. If
        # pwm_lookup() is coded to interpolate between voltages, then this function could be set to a precise
        # measured voltage
        self.thruster_voltages = [16, 16, 16, 16, 16, 16, 16, 16]

        # thrust current force. This is the force a thruster is currently projecting as a percentage of its max force.
        # this number should remain between -1 and 1.
        self.thruster_current_pwm = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

    # this function will be filled in to get a pwm value based on the desired force and available voltage
    def pwm_lookup(self, voltage, force):
        # this function should be able to interpolate between forces. Ideally, this function could also interpolate
        # between voltages
        return 1500  # set as placeholder

    # this function sets a thruster to the correct pwm for a desired voltage
    def set_pwm(self, thruster, force):
        if thruster not in range(0, 8):
            raise ValueError("thruster does not exist")
        pwm = self.pwm_lookup(self.thruster_voltages[thruster], force)
        self.thruster_current_pwm[thruster] = pwm

    # returns the expected drag force on the sub based on velocity vector. Writing this function
    # accurately may require experimentation. Currently, Making this function accurate will
    # be very complicated, but a very simple drag estimate, which may be entirely proportional to velocity, will be a
    # good start
    def expected_drag_force(self):
        pass

    def get_weight_force(self):
        # this function needs to be expanded give th weight force vector at different orientation, and to give the
        # torque if the weight force vector does not intersect the robots center of gravity
        w = np.matrix([0, 0, self.force_weight, 0, 0, 0])
        return w

    # solves for environmental forces
    def solve_e(self):
        pass

    # solves thruster direction matrix
    def get_d(self):
        sin_45: float = 2 ** (-1 / 2)
        # rows 1, 2, and 3 represent linear force and should be fairly static. These are proportional to the total
        # force of each thruster rows 4, 5, and 6 represent moments about the robots center of mass. These should be
        # calculated based on thruster positions, but for now they are static. They should be proportional to an
        # arbitrary measure of torque, but for now they are not
        dFx = np.array([0, 0, 0, 0, sin_45, sin_45, sin_45, sin_45])
        dFy = np.array([0, 0, 0, 0, sin_45, -sin_45, -sin_45, sin_45])
        dFz = np.array([1, 1, 1, 1, 0, 0, 0, 0])
        dMx = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        dMy = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        dMz = np.array([0, 0, 0, 0, -1, 1, -1, 1])
        return np.array([dFx, dFy, dFz, dMx, dMy, dMz])

    # Fx, Fy, Fz, Mz
    def get_simple_d(self):
        return np.delete(self.get_d(), [3, 4])

    # keeps sub steady, ignoring Mx and My, accounting for environmental factors
    def simple_stabilize(self):
        pass

    def simple_vetical(self):
        pass

    def simple_horizantal(self):
        pass

