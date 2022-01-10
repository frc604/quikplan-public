import os
import json

import matplotlib as mpl
import matplotlib.animation as animation
import numpy as np
import pylab as plt

PATH_OBSTACLES = {
    "barrel-racing": ["B1", "B2", "B8", "D1", "D2", "D5", "D10"],
    "slalom": ["B1", "B2", "D1", "D2", "D4", "D5", "D6", "D7", "D8", "D10"],
    "slalom-simplified": ["B2", "D2", "D4", "D8", "D10"],
}

DEFAULT_OBSTACLE_DIA_IN = 3  # inches


def in2m(inches):
    # Inches to meters
    return inches * 2.54 / 100


def rotate_around_origin(point, theta):
    x, y = point
    return (
        x * np.cos(theta) - y * np.sin(theta),
        y * np.cos(theta) + x * np.sin(theta),
    )


def transform_geometry(geometry, pose):
    x, y, theta = pose
    transformed_geometry = []
    for point1, point2 in geometry:
        new_point1 = rotate_around_origin(point1, theta) + np.array([x, y])
        new_point2 = rotate_around_origin(point2, theta) + np.array([x, y])
        transformed_geometry.append((new_point1, new_point2))
    return transformed_geometry


def plot_robot(ax, pose, robot_geometry, robot_axis_plot_size):
    # Plot robot geometry
    ax.add_collection(
        mpl.collections.LineCollection(
            transform_geometry(robot_geometry, pose), color="k"
        )
    )
    # Plot robot axes
    ax.add_collection(
        mpl.collections.LineCollection(
            transform_geometry([[(0, 0), (robot_axis_plot_size, 0)]], pose),
            color="r",
        )
    )
    ax.add_collection(
        mpl.collections.LineCollection(
            transform_geometry([[(0, 0), (0, robot_axis_plot_size)]], pose),
            color="g",
        )
    )


def plot_traj(
    title,
    xs,
    ys,
    thetas,
    obstacles,
    robot_geometry,
    robot_axis_plot_size,
    targets=[],
    goal=None,
    limits=(-1, in2m(360) + 1, -1, in2m(180) + 1),
    draw_bounds=True,
    invert=False,
    robot_plot_mod=5,
    save_png=False,
):
    plt.style.use("ggplot")
    fig, ax = plt.subplots()
    if draw_bounds:
        ax.add_patch(
            mpl.patches.Rectangle(
                (0, 0),
                in2m(360),
                in2m(180),
                linewidth=1,
                edgecolor="k",
                facecolor="none",
            )
        )
    for obx, oby, obr in obstacles:
        ax.add_artist(plt.Circle((obx, oby), obr, color="r"))
    for tx, ty, tr in targets:
        ax.add_artist(plt.Circle((tx, ty), tr, color="g"))
    if goal is not None:
        ax.add_collection(
            mpl.collections.LineCollection(
                [goal],
                color="r",
                linewidths=4,
            )
        )
    plt.scatter(xs, ys, marker=".", color="#FAB604")
    for i, pose in enumerate(zip(xs, ys, thetas)):
        if robot_plot_mod is None:
            if i == 0:
                plot_robot(ax, pose, robot_geometry, robot_axis_plot_size)
        else:
            if i % robot_plot_mod == 0:
                plot_robot(ax, pose, robot_geometry, robot_axis_plot_size)
        # Always plot last pose
        if i == len(xs) - 1:
            plot_robot(ax, pose, robot_geometry, robot_axis_plot_size)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.xlim(limits[0], limits[1])
    plt.ylim(limits[2], limits[3])
    plt.gca().set_aspect("equal", adjustable="box")
    plt.title(title)

    if invert:
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
    if save_png:
        if not os.path.exists("renders"):
            os.makedirs("renders")
        plt.savefig(
            os.path.join("renders", "{}.png".format(title)), transparent=True, dpi=800
        )


def anim_traj(
    title,
    xs,
    ys,
    thetas,
    obstacles,
    robot_geometry,
    robot_axis_plot_size,
    timestep,
    invert=False,
    targets=[],
    goal=None,
    limits=(-1, in2m(360) + 1, -1, in2m(180) + 1),
    draw_bounds=True,
    save_gif=False,
):
    fig, ax = plt.subplots()

    num_states = len(xs)
    if draw_bounds:
        if draw_bounds:
            ax.add_patch(
                mpl.patches.Rectangle(
                    (0, 0),
                    in2m(360),
                    in2m(180),
                    linewidth=1,
                    edgecolor="k",
                    facecolor="none",
                )
            )
    for obx, oby, obr in obstacles:
        ax.add_artist(plt.Circle((obx, oby), obr, color="r"))
    for tx, ty, tr in targets:
        ax.add_artist(plt.Circle((tx, ty), in2m(6), color="g"))
    if goal is not None:
        ax.add_collection(
            mpl.collections.LineCollection(
                [goal],
                color="r",
                linewidths=4,
            )
        )
    plt.scatter(xs, ys, marker=".")

    # Plot first pose
    plot_robot(ax, (xs[0], ys[0], thetas[0]), robot_geometry, robot_axis_plot_size)

    # Plot last pose
    plot_robot(
        ax,
        (xs[num_states - 1], ys[num_states - 1], thetas[num_states - 1]),
        robot_geometry,
        robot_axis_plot_size,
    )

    # Animation function
    def animate(i):
        if save_gif:
            print("Rendering Frame: {}".format(i))
        pose = list(zip(xs, ys, thetas))[i]
        # Hack to remove the old robot poses
        ax.collections = ax.collections[:7]

        plot_robot(ax, pose, robot_geometry, robot_axis_plot_size)
        return ax.collections

    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.xlim(limits[0], limits[1])
    plt.ylim(limits[2], limits[3])
    plt.gca().set_aspect("equal", adjustable="box")
    plt.title(title)

    if invert:
        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()

    anim = animation.FuncAnimation(
        fig, animate, frames=num_states, interval=timestep, blit=True, repeat=True
    )

    if save_gif:
        if not os.path.exists("renders"):
            os.makedirs("renders")
        anim.save(
            os.path.join("renders", "{}.gif".format(title)),
            writer="pillow",
            dpi=100,
            fps=(int)(1 / (timestep / 1000)),
        )
        save_gif = False
    return anim


def load_init_json(init_json, start_pose, num_states):
    x1, y1, theta1 = start_pose

    with open(init_json) as f:
        init_traj = json.load(f)
    init_traj_len = len(init_traj)
    n_per_waypoint = int((num_states + 1) / init_traj_len)

    x_init = []
    y_init = []
    theta_init = []
    for i in range(init_traj_len):
        x2, y2 = init_traj[i]
        theta2 = np.arctan2(y2 - y1, x2 - x1)
        theta2 = np.unwrap([theta1, theta2])[-1]

        x_init += list(np.linspace(x1, x2, n_per_waypoint))
        y_init += list(np.linspace(y1, y2, n_per_waypoint))
        theta_init += list(np.linspace(theta1, theta2, n_per_waypoint))

        x1, y1, theta1 = x2, y2, theta2

    # Fill in remainder, if any
    remainder = num_states - len(x_init) + 1
    x_init += [x2] * remainder
    y_init += [y2] * remainder
    theta_init += [theta2] * remainder

    return x_init, y_init, theta_init


def plot_wheel_vel_accel_jerk(times, vl, vr, al, ar, jl, jr):
    plt.plot(
        times,
        vl,
        label="Left Wheel Velocity",
        linewidth=4,
        color="red",
    )
    plt.plot(
        times,
        vr,
        label="Right Wheel Velocity",
        linewidth=4,
        color="blue",
    )
    plt.plot(
        times,
        al,
        label="Left Wheel Acceleration",
        color="firebrick",
    )
    plt.plot(
        times,
        ar,
        label="Right Wheel Acceleration",
        color="royalblue",
    )
    plt.plot(
        times,
        jl,
        label="Left Wheel Jerk",
        linestyle="--",
        color="lightcoral",
    )
    plt.plot(
        times,
        jr,
        label="Right Wheel Jerk",
        linestyle="--",
        color="cornflowerblue",
    )
    plt.legend(loc="lower left")
    plt.xlabel("Time (s)")


def plot_wheel_forces(times, lon_fl, lon_fr, lat_f):
    plt.plot(times, lon_fl, label="Longitudinal Force (Left)")
    plt.plot(times, lon_fr, label="Longitudinal Force (Right)")
    plt.plot(times, lat_f, label="Centripetal Force")
    plt.legend(loc="lower left")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")


def plot_total_force(times, total_force_left, total_force_right):
    plt.plot(
        times,
        total_force_left,
        label="Total Force (Left)",
    )
    plt.plot(
        times,
        total_force_right,
        label="Total Force (Right)",
    )
    plt.legend(loc="lower left")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")


def interp_state_vector(times, states, new_dt):
    interp_times = np.arange(0, times[-1], new_dt)
    return np.interp(interp_times, times, states[:-1])


def ids2obstales(ids, diameter):
    obstacles = []
    for id in ids:
        id = id.upper()
        row = ord(id[0]) - 64
        col = int(id[1:])
        x = in2m(30) * col
        y = in2m(180) - row * in2m(30)
        obstacles.append((x, y, diameter / 2))
    return obstacles


def create_obstacles(pathname, diameter_inches=DEFAULT_OBSTACLE_DIA_IN):
    return ids2obstales(PATH_OBSTACLES[pathname], in2m(diameter_inches))


def besph2cart(bearing: float, elevation: float):
    """Converts a bearing and elevation in spherical coordinates to a 3D cartesian point on the unit circle."""
    return np.array(
        [
            np.cos(bearing) * np.cos(elevation),
            np.sin(bearing) * np.cos(elevation),
            np.sin(elevation),
        ]
    )


def cart2besph(x: float, y: float, z: float):
    """Converts the direction of a 3D cartesian point to a bearing and elevation in spherical coordinates."""
    return np.array(
        [np.arctan2(y, x), np.arctan2(z, np.sqrt(np.power(x, 2) + np.power(y, 2)))]
    )


class LineBuilder:
    def __init__(self, line):
        self.line = line
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())

        self.dragging = None
        self.connect()

    def connect(self):
        self.cidpress = self.line.figure.canvas.mpl_connect(
            "button_press_event", self.on_press
        )
        self.cidrelease = self.line.figure.canvas.mpl_connect(
            "button_release_event", self.on_release
        )
        self.cidmotion = self.line.figure.canvas.mpl_connect(
            "motion_notify_event", self.on_motion
        )

    def disconnect(self):
        self.line.figure.canvas.mpl_disconnect(self.cidpress)
        self.line.figure.canvas.mpl_disconnect(self.cidrelease)
        self.line.figure.canvas.mpl_disconnect(self.cidmotion)

    def on_press(self, event):
        if event.inaxes != self.line.axes:
            return

        point_threshold = 0.1

        # Find nearest point to cursor
        min_distance = float("inf")
        for i, point in enumerate(zip(self.xs, self.ys)):
            x, y = point
            distance = np.sqrt((event.xdata - x) ** 2 + (event.ydata - y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        if event.button == 1:
            if min_distance < point_threshold:
                self.dragging = closest_index
            else:
                self.xs.append(event.xdata)
                self.ys.append(event.ydata)
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()
        elif event.button == 3:
            if min_distance < point_threshold:
                self.xs.pop(closest_index)
                self.ys.pop(closest_index)
                self.line.set_data(self.xs, self.ys)
                self.line.figure.canvas.draw()

    def on_motion(self, event):
        if self.dragging is None:
            return
        if event.inaxes != self.line.axes:
            return

        self.xs[self.dragging] = event.xdata
        self.ys[self.dragging] = event.ydata
        self.line.set_data(self.xs, self.ys)
        self.line.figure.canvas.draw()

    def on_release(self, event):
        self.dragging = None
        self.line.figure.canvas.draw()
