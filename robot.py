import casadi as ca
import numpy as np

from constants import ControlVars, G, StateVars
from helpers import in2m, rotate_around_origin

DEFAULT_MU = 0.9  # Coefficient of friction
DEFAULT_MAX_TORQUE = 0.5  # N*m @ 40A
DEFAULT_MAX_RPM = 4700  # RPM @ 10V
DEFAULT_OBSTACLE_BUFFER = 0.2  # m


class Robot(object):
    def __init__(self):
        # Geometry
        self.WIDTH = in2m(34)  # m
        self.LENGTH = in2m(39)  # m
        self.TRACK_WIDTH = 0.7269198037390904  # m
        self.WHEEL_DIA = in2m(6)  # m
        self.GEOMETRY = [
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Front
            [
                (-self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Back
            [
                (self.LENGTH / 2, self.WIDTH / 2),
                (-self.LENGTH / 2, self.WIDTH / 2),
            ],  # Left
            [
                (self.LENGTH / 2, -self.WIDTH / 2),
                (-self.LENGTH / 2, -self.WIDTH / 2),
            ],  # Right
        ]
        self.AXIS_SIZE = self.WIDTH / 4
        self.OBSTACLE_BUFFER = DEFAULT_OBSTACLE_BUFFER

        # Mass / Inertia
        self.MU = DEFAULT_MU
        self.MASS = 50.0  # kg
        self.J = (
            self.MASS * (self.LENGTH ** 2 + self.WIDTH ** 2)
        ) / 12.0  # Moment of inertia

        # Motors
        self.MOTORS_PER_SIDE = 2
        self.MOTOR_MAX_TORQUE = DEFAULT_MAX_TORQUE
        self.MOTOR_MAX_RPM = DEFAULT_MAX_RPM
        self.DRIVE_RATIO = 9.5625  # 9.5625:1
        self.WHEEL_40A_FORCE = (
            self.MOTORS_PER_SIDE
            * self.MOTOR_MAX_TORQUE
            * self.DRIVE_RATIO
            / (self.WHEEL_DIA / 2)
        )

    def dynamics_model(self, x, u):
        # dx/dt = f(x, u)
        theta = x[StateVars.thetaIdx.value]
        vl = x[StateVars.vlIdx.value]
        vr = x[StateVars.vrIdx.value]
        al = x[StateVars.alIdx.value]
        ar = x[StateVars.arIdx.value]
        jl = u[ControlVars.jlIdx.value]
        jr = u[ControlVars.jrIdx.value]

        v = (vl + vr) / 2

        return ca.vertcat(
            v * np.cos(theta),  # x_dot
            v * np.sin(theta),  # y_dot
            (vr - vl) / self.TRACK_WIDTH,  # theta_dot
            al,  # vl_dot
            ar,  # vr_dot
            jl,  # al_dot
            jr,  # ar_dot
        )

    def get_longitudinal_wheel_forces(self, al, ar):
        # Solve this system of equations for |lon_fl| and |lon_fr|
        # a = (lon_fl + lon_fr) / MASS
        # R = TRACK_WIDTH / 2
        # al = a - w_dot * R
        # ar = a + w_dot * R
        # w_dot = (lon_fr - lon_fl) * R / J
        R2 = (self.TRACK_WIDTH / 2) ** 2
        lon_fl = (self.J * (al - ar) + self.MASS * R2 * (al + ar)) / (4 * R2)
        lon_fr = (self.J * (ar - al) + self.MASS * R2 * (al + ar)) / (4 * R2)
        return lon_fl, lon_fr

    def get_lateral_wheel_force(self, vl, vr):
        # lat_f = MASS * v^2 * curvature
        # lat_f = MASS * v^2 * (w / v)
        # lat_f = MASS * v * w
        v = (vl + vr) / 2
        w = (vr - vl) / self.TRACK_WIDTH
        lat_f = self.MASS * v * w
        return lat_f

    def apply_wheel_constraints(self, opti, vl, vr, al, ar, jl, jr):
        # Note that acceleration limits are added here to help with convergence,
        # but are large enough to not be restrictive.
        # A more realistic acceleration limit is applied by wheel force constraints later.
        max_v = (self.MOTOR_MAX_RPM / 60 / self.DRIVE_RATIO) * np.pi * self.WHEEL_DIA
        opti.subject_to(opti.bounded(-max_v, vl, max_v))  # Left wheel velocity limit
        opti.subject_to(opti.bounded(-max_v, vr, max_v))  # Right wheel velocity limit
        opti.subject_to(opti.bounded(-8, al, 8))  # Left wheel acceleration limit
        opti.subject_to(opti.bounded(-8, ar, 8))  # Right wheel acceleration limit
        opti.subject_to(opti.bounded(-20, jl, 20))  # Left wheel jerk limit
        opti.subject_to(opti.bounded(-20, jr, 20))  # Right wheel jerk limit

    def apply_wheel_force_constraints(self, opti, al, ar):
        lon_fl, lon_fr = self.get_longitudinal_wheel_forces(al, ar)

        # Apply motor torque (converted into force) constraints
        opti.subject_to(
            opti.bounded(-self.WHEEL_40A_FORCE, lon_fl, self.WHEEL_40A_FORCE)
        )
        opti.subject_to(
            opti.bounded(-self.WHEEL_40A_FORCE, lon_fr, self.WHEEL_40A_FORCE)
        )

    def apply_wheel_friction_constraints(self, opti, vl, vr, al, ar):
        lon_fl, lon_fr = self.get_longitudinal_wheel_forces(al, ar)
        lat_f = self.get_lateral_wheel_force(vl, vr)

        # Apply friction limit constraints (expanded out to avoid sqrt)
        friction_limit_sq = (self.MU * G * (self.MASS / 2)) ** 2
        opti.subject_to(lon_fl ** 2 + lat_f ** 2 <= friction_limit_sq)
        opti.subject_to(lon_fr ** 2 + lat_f ** 2 <= friction_limit_sq)

    def apply_obstacle_constraints(self, opti, xpos, ypos, theta, obstacles):
        for obx, oby, obr in obstacles:
            for p1, p2 in self.GEOMETRY:
                # Transform robot geometry to pose
                x1, y1 = rotate_around_origin(p1, theta)
                x2, y2 = rotate_around_origin(p2, theta)
                x1 += xpos
                y1 += ypos
                x2 += xpos
                y2 += ypos

                # Compute the closest distance between a point and a line segment
                px = x2 - x1
                py = y2 - y1
                norm = px * px + py * py
                u = ((obx - x1) * px + (oby - y1) * py) / norm
                u = ca.fmax(ca.fmin(u, 1), 0)
                x = x1 + u * px
                y = y1 + u * py
                dx = x - obx
                dy = y - oby

                dist = np.sqrt((dx * dx + dy * dy))
                opti.subject_to(dist > obr + self.OBSTACLE_BUFFER)

    def apply_finish_line_constraints(
        self, opti, endx, endy, endtheta, finish_line, direction, backwards=False
    ):
        # Transform robot geometry to pose
        # Only need to consider front or back edge, depending on direction
        p1, p2 = self.GEOMETRY[1] if backwards else self.GEOMETRY[0]
        x1, y1 = rotate_around_origin(p1, endtheta)
        x2, y2 = rotate_around_origin(p2, endtheta)
        x1 += endx
        y1 += endy
        x2 += endx
        y2 += endy

        # Enforce that at least one corner crosses the line
        (x1_fin, y1_fin), (x2_fin, y2_fin) = finish_line
        if direction in ["right", "left"]:
            assert x1_fin == x2_fin

            if direction == "right":
                opti.subject_to(ca.fmax(x1, x2) > x1_fin)
            else:
                opti.subject_to(ca.fmin(x1, x2) < x1_fin)
        elif direction in ["up", "down"]:
            assert y1_fin == y2_fin

            if direction == "up":
                opti.subject_to(ca.fmax(y1, y2) > y1_fin)
            else:
                opti.subject_to(ca.fmin(y1, y2) < y1_fin)
        else:
            raise Exception(f"Unknown direction '{direction}")
