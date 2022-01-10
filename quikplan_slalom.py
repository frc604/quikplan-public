#!/usr/bin/env python3

import casadi as ca
import numpy as np
import pylab as plt

from constants import ControlVars, StateVars
from helpers import (
    load_init_json,
    anim_traj,
    create_obstacles,
    in2m,
    interp_state_vector,
    plot_traj,
    rotate_around_origin,
    plot_wheel_vel_accel_jerk,
    plot_wheel_forces,
    plot_total_force,
)
from robot import Robot


def plan(robot=Robot(), plot=False):
    N = 200  # Number of control intervals

    OBSTACLES = create_obstacles("slalom-simplified")
    FINISH_LINE_BUFFER = 0.1

    # Setup Optimization
    opti = ca.Opti()

    # State variables
    X = opti.variable(len(StateVars), N + 1)
    xpos = X[StateVars.xIdx.value, :]  # X position
    ypos = X[StateVars.yIdx.value, :]  # Y-position
    theta = X[StateVars.thetaIdx.value, :]  # Theta
    vl = X[StateVars.vlIdx.value, :]  # Left wheel velocity
    vr = X[StateVars.vrIdx.value, :]  # Right wheel velocity
    al = X[StateVars.alIdx.value, :]  # Left wheel acceleration
    ar = X[StateVars.arIdx.value, :]  # Right wheel acceleration

    # Control variables
    U = opti.variable(len(ControlVars), N)
    jl = U[ControlVars.jlIdx.value, :]  # Left wheel jerk
    jr = U[ControlVars.jrIdx.value, :]  # Right wheel jerk

    # Total time variable
    T = opti.variable()
    dt = T / N  # length of one control interval

    # Minimize time
    opti.minimize(T)

    # Apply dynamic constriants
    for k in range(N):
        x_next = X[:, k] + robot.dynamics_model(X[:, k], U[:, k]) * dt
        opti.subject_to(X[:, k + 1] == x_next)

    # Wheel constraints
    robot.apply_wheel_constraints(opti, vl, vr, al, ar, jl, jr)

    # Boundary conditions
    # Start
    opti.subject_to(xpos[0] == in2m(60) - robot.LENGTH / 2)
    opti.subject_to(ypos[0] == in2m(30))
    opti.subject_to(theta[0] == 0)
    opti.subject_to(vl[0] == 0)
    opti.subject_to(vr[0] == 0)
    opti.subject_to(al[0] == 0)
    opti.subject_to(ar[0] == 0)
    opti.subject_to(jl[0] == 0)
    opti.subject_to(jr[0] == 0)

    # End
    robot.apply_finish_line_constraints(
        opti,
        xpos[-1],
        ypos[-1],
        theta[-1],
        (
            (in2m(60) - FINISH_LINE_BUFFER, in2m(60)),
            (in2m(60) - FINISH_LINE_BUFFER, in2m(120)),
        ),
        "left",
    )

    # Obstacles
    robot.apply_obstacle_constraints(opti, xpos, ypos, theta, OBSTACLES)

    # Time constraints
    opti.subject_to(T >= 0)

    # Compute initial guess from init traj
    x_init, y_init, theta_init = load_init_json(
        "init_traj/slalom.json", (in2m(30), in2m(30), 0.0), N
    )

    # Initial guess
    opti.set_initial(xpos, x_init)
    opti.set_initial(ypos, y_init)
    opti.set_initial(theta, theta_init)
    opti.set_initial(vl, 0)
    opti.set_initial(vr, 0)
    opti.set_initial(al, 0)
    opti.set_initial(ar, 0)
    opti.set_initial(jl, 0)
    opti.set_initial(jr, 0)
    opti.set_initial(T, 10)

    if plot:
        # Plot initialization
        plot_traj(
            "Initial Trajectory",
            x_init,
            y_init,
            theta_init,
            OBSTACLES,
            robot.GEOMETRY,
            robot.AXIS_SIZE,
        )

    # Solve non-linear program
    opti.solver("ipopt", {}, {"mu_init": 1e-3})  # set numerical backend
    sol = opti.solve()

    if plot:
        # Plot result without wheel force limits
        plot_traj(
            "Before Wheel Force Limits",
            sol.value(xpos),
            sol.value(ypos),
            sol.value(theta),
            OBSTACLES,
            robot.GEOMETRY,
            robot.AXIS_SIZE,
        )

    # Solve the problem again, but this time with wheel force & friction limit constraints
    robot.apply_wheel_force_constraints(opti, al, ar)
    robot.apply_wheel_friction_constraints(opti, vl, vr, al, ar)

    # Copy over X, U, and T to initialize
    opti.set_initial(X, sol.value(X))
    opti.set_initial(U, sol.value(U))
    opti.set_initial(T, sol.value(T))
    sol = opti.solve()

    times = np.linspace(0, sol.value(T), N)

    if plot:
        # Plot final result
        plot_traj(
            "Final Result",
            sol.value(xpos),
            sol.value(ypos),
            sol.value(theta),
            OBSTACLES,
            robot.GEOMETRY,
            robot.AXIS_SIZE,
        )

        plt.figure()
        plot_wheel_vel_accel_jerk(
            times,
            sol.value(vl)[:-1],
            sol.value(vr)[:-1],
            sol.value(al)[:-1],
            sol.value(ar)[:-1],
            sol.value(jl),
            sol.value(jr),
        )

        lon_fl, lon_fr = robot.get_longitudinal_wheel_forces(al, ar)
        lat_f = robot.get_lateral_wheel_force(vl, vr)

        plt.figure()
        plot_wheel_forces(
            times,
            sol.value(lon_fl)[:-1],
            sol.value(lon_fr)[:-1],
            sol.value(lat_f)[:-1],
        )

        plt.figure()
        plot_total_force(
            times,
            np.sqrt(sol.value(lon_fl) ** 2 + sol.value(lat_f) ** 2)[:-1],
            np.sqrt(sol.value(lon_fr) ** 2 + sol.value(lat_f) ** 2)[:-1],
        )

        interp_time = 0.02  # seconds

        interp_x = interp_state_vector(times, sol.value(xpos), interp_time)
        interp_y = interp_state_vector(times, sol.value(ypos), interp_time)
        interp_theta = interp_state_vector(times, sol.value(theta), interp_time)

        plot_traj(
            "Interp",
            interp_x,
            interp_y,
            interp_theta,
            OBSTACLES,
            robot.GEOMETRY,
            robot.AXIS_SIZE,
            save_png=True,
        )

        anim = anim_traj(
            "Final Result",
            interp_x,
            interp_y,
            interp_theta,
            OBSTACLES,
            robot.GEOMETRY,
            robot.AXIS_SIZE,
            20,  # milliseconds
            save_gif=False,
        )

        plt.show()

    print(f"Trajectory time: {sol.value(T)} seconds")


if __name__ == "__main__":
    robot = Robot()
    plan(robot, plot=True)
