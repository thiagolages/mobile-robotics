import time
from pprint import pprint

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

matplotlib.use("TkAgg")  # Use Agg backend for headless operation


def build_object_dict(sim, excluded_objects):
    obj_handles = get_frame_handles(sim, excluded_objects)
    obj_names = get_frame_names(sim, obj_handles)
    obj_poses = get_frame_poses(sim, obj_handles)
    assert obj_handles is not None
    assert obj_names is not None
    assert obj_poses is not None
    dict = {
        obj_name: {"handle": obj_handle, "pose": obj_pose}
        for obj_name, obj_handle, obj_pose in zip(
            obj_names, obj_handles, obj_poses
        )
    }
    return dict


def get_frame_handles(sim, excluded_objects=None, verbose=False):
    if excluded_objects is None:
        print("No excluded objects provided")
    handles = [
        # 0 means no options enabled. From the docs:
        # bit0 set (1): exclude the tree base from the returned array
        # bit1 set (2): include in the returned array only the object's first
        # children. If treeBaseHandle is sim.handle_scene, then only parentless
        # objects will be included.
        # sim.sceneobject_dummy means to get only dummies, not other types
        handle
        for handle in sim.getObjectsInTree(
            sim.handle_scene, sim.sceneobject_dummy, 0
        )
        if (
            sim.getObjectAlias(handle) not in excluded_objects
            and "_frame" in sim.getObjectAlias(handle)  # noqa: W503
        )
    ]
    print(f"Found {len(handles)} handles: {handles}")

    return handles


def get_frame_names(sim, handles):
    names = []
    for handle in handles:
        print(f"Getting name for handle = {handle}")
        names.append(sim.getObjectAlias(handle))
        print(f"Name: {sim.getObjectAlias(handle)}")

    return names


def get_frame_poses(sim, handles):
    poses = []
    for handle in handles:
        print(f"Getting pose for handle = {handle}")
        poses.append(sim.getObjectPose(handle))

    return poses


def print_dict_info(obj_dict):
    for name, obj in obj_dict.items():
        pose = obj["pose"]
        # Print pose with at most 2 decimal places
        formatted_pose = [
            round(x, 2) if isinstance(x, float) else x for x in pose
        ]
        pprint(
            f"ID: {id} | Name: {name} | Pose: {formatted_pose}",
            indent=4,
            width=80,
            compact=True,
        )
        # print(f"ID: {id} | Name: {name} | Pose: {formatted_pose}")
        print("-" * 40)

    print(f"Finished getting object poses. Total objects: {len(obj_dict)}")


def tf_from_pose(sim, pose: list[float]) -> np.ndarray:
    """
    Convert a pose to a homogeneous transformation matrix.
    The pose is a list of 7 elements: [x, y, z, qx, qy, qz, qw]
    The homogeneous transformation matrix is a 4 x 4 matrix.
    """
    # list of 12 elements [Vx0 Vy0 Vz0 P0 Vx1 Vy1 Vz1 P1 Vx2 Vy2 Vz2 P2]
    tf = np.array(sim.poseToMatrix(pose))
    tf = tf.reshape(3, 4)  # 3 x 4 matrix
    tf = np.vstack((tf, np.array([[0, 0, 0, 1]])))  # HTM

    return tf


def get_tf(
    sim, handle: int, inv: bool = False, verbose: bool = False
) -> np.ndarray:
    """
    Gets the tf from the world to the obj by default.
    If 'inv' is True, gets the inverse transform.
    """
    handle_name = sim.getObjectAlias(handle)
    ref_frame = sim.handle_inverse if inv else sim.handle_world
    pose = sim.getObjectPose(handle, ref_frame)  # (x, y, z, qx, qy, qz, qw)
    tf = tf_from_pose(sim, pose)
    if verbose:
        print(
            "Transform from {} to {}: {}".format(
                handle_name if inv else "world",
                "world" if inv else handle_name,
                tf,
            )
        )

    return tf


def handle_sim_start(sim):

    state = sim.getSimulationState()

    if state == sim.simulation_stopped:
        print("Starting simulation")
        sim.startSimulation()
    elif state == sim.simulation_paused:
        print("Simulation is paused, stopping it and then starting it")
        sim.stopSimulation()
        time.sleep(1)
        print("Starting simulation")
        sim.startSimulation()
    else:
        sim.stopSimulation()
        time.sleep(1)
        print("Starting simulation")
        sim.startSimulation()


def plot_frame(ax, T, label, frame_size=1.0):
    """
    Plot a 3D coordinate frame given a homogeneous transformation matrix.

    Args:
        ax: matplotlib 3D axis
        T: 4x4 homogeneous transformation matrix
        label: label for the frame
        frame_size: size of the frame axes
    """
    # Extract position and rotation from transformation matrix
    position = T[:3, 3]
    rotation = T[:3, :3]

    # Define unit vectors for X, Y, Z axes
    x_axis = rotation @ np.array([frame_size, 0, 0])
    y_axis = rotation @ np.array([0, frame_size, 0])
    z_axis = rotation @ np.array([0, 0, frame_size])

    # Plot the axes
    # X-axis in red
    ax.quiver(
        position[0],
        position[1],
        position[2],
        x_axis[0],
        x_axis[1],
        x_axis[2],
        color="red",
        arrow_length_ratio=0.1,
        linewidth=2,
    )

    # Y-axis in green
    ax.quiver(
        position[0],
        position[1],
        position[2],
        y_axis[0],
        y_axis[1],
        y_axis[2],
        color="green",
        arrow_length_ratio=0.1,
        linewidth=2,
    )

    # Z-axis in blue
    ax.quiver(
        position[0],
        position[1],
        position[2],
        z_axis[0],
        z_axis[1],
        z_axis[2],
        color="blue",
        arrow_length_ratio=0.1,
        linewidth=2,
    )

    # Add label near the frame origin
    ax.text(position[0], position[1], position[2], f"  {label}", fontsize=8)


def plot_robot_to_object_lines(
    ax, robot_to_obj_tfs, robot_name, line_color="orange", line_alpha=0.7
):
    """
    Plot arrows from robot to each object pose.

    Args:
        ax: matplotlib 3D axis
        robot_to_obj_tfs: dictionary of robot-to-object transforms
        robot_name: name of the robot (to skip robot frames)
        line_color: color of the arrows
        line_alpha: transparency of the arrows
    """
    robot_origin = np.array([0, 0, 0])  # Robot is at origin

    for name, T_robot_obj in robot_to_obj_tfs.items():
        if robot_name in name:
            continue  # Skip robot frames

        # Get object position relative to robot
        obj_position = T_robot_obj[:3, 3]

        # Calculate direction vector from robot to object
        direction = obj_position - robot_origin

        # Plot arrow from robot to object using quiver
        ax.quiver(
            robot_origin[0],
            robot_origin[1],
            robot_origin[2],
            direction[0],
            direction[1],
            direction[2],
            color=line_color,
            alpha=line_alpha,
            linewidth=2,
            arrow_length_ratio=0.1,
            length=1.0,
        )

        # Add transform name label at midpoint
        midpoint = (robot_origin + obj_position) / 2
        # Plot transform name
        ax.text(
            midpoint[0],
            midpoint[1],
            midpoint[2],
            f'T_r_{name.split("_")[0]}',
            fontsize=6,
            ha="center",
            va="center",
            bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7),
        )
        # Plot distance
        # distance = np.linalg.norm(obj_position)
        # ax.text(midpoint[0], midpoint[1], midpoint[2],
        #         f'{distance:.2f}m', fontsize=6, ha='center', va='center',
        #         bbox=dict(
        #             boxstyle="round,pad=0.2",
        #             facecolor='white',
        #             alpha=0.7
        #         ))


def plot_robot_to_object_tfs(
    robot_name, robot_to_obj_tfs, save_path=None, camera_angle=None
):
    """
    Plot 3D frames showing the robot-to-object transforms.
    Robot frame is at origin (0,0,0) with identity orientation.

    Args:
        robot_name: name of the robot
        robot_to_obj_tfs: dictionary of robot-to-object transforms
        save_path: path to save the main plot (default: "robot_frames_3d.png")
        camera_angle: tuple (elevation, azimuth) for specific camera angle
    """
    if save_path is None:
        save_path = "robot_frames_3d.png"

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")
    frame_size = 0.5

    # Plot robot frame at origin with identity orientation
    robot_tf = np.eye(4)  # Identity matrix - robot at origin
    plot_frame(ax, robot_tf, robot_name, frame_size=frame_size)

    # Plot frames for each object relative to robot
    for name, T_robot_obj in robot_to_obj_tfs.items():
        if robot_name in name:
            print(f"Skipping robot frame for {name}")
            continue
        print(f"Plotting transform from robot to {name}: \n{T_robot_obj}")
        plot_frame(ax, T_robot_obj, name, frame_size=frame_size)
        print("-" * 40)

    # Plot lines from robot to objects
    plot_robot_to_object_lines(ax, robot_to_obj_tfs, robot_name)

    # Set up the plot
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot-to-Object Transforms with Connection Lines")

    # Set equal aspect ratio and reasonable limits
    max_range = 3.0  # Adjust based on your scene size
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])

    # Make the axes equal
    ax.set_box_aspect([1, 1, 1])

    # Add a grid
    ax.grid(True, alpha=0.3)

    # Add legend
    legend_elements = [
        Line2D([0], [0], color="red", lw=2, label="X-axis"),
        Line2D([0], [0], color="green", lw=2, label="Y-axis"),
        Line2D([0], [0], color="blue", lw=2, label="Z-axis"),
        Line2D(
            [0],
            [0],
            color="orange",
            lw=2,
            linestyle="--",
            label="Robot-Object Arrows",
        ),
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()

    # Save the main plot
    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    print(f"3D frame plot saved as '{save_path}'")

    # Save additional camera angles if specified
    if camera_angle:
        elev, azim, roll = camera_angle

        # Set camera view
        ax.view_init(elev=elev, azim=azim, roll=roll)
        print(f"Set camera view to elev={elev}°, azim={azim}°, roll={roll}°")

    # Save
    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    print(f"Saved as '{save_path}'")

    plt.show()
    plt.close()


def generate_random_pose(
    sim, xlim=[-1, 1], ylim=[-1, 1], zlim=[-1, 1], theta_lim=[-1, 1]
):
    """
    Generate a random pose for the robot.
    Returns a homogeneous transformation matrix.
    """
    pose = [
        1,
        0,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        0,
        1,
        0,
    ]  # 12 elements, identity rotation, no translation

    axis = [0, 0, 1]
    axisPos = [0, 0, 0]
    # Rotate around the random unit vector
    pose = sim.rotateAroundAxis(
        pose, axis, axisPos, np.random.uniform(theta_lim[0], theta_lim[1])
    )
    pose = np.array(pose).reshape(3, 4)
    # Set the position
    pose[0, 3] = np.random.uniform(xlim[0], xlim[1])
    pose[1, 3] = np.random.uniform(ylim[0], ylim[1])
    pose[2, 3] = np.random.uniform(zlim[0], zlim[1])

    pose = np.vstack((pose, np.array([[0, 0, 0, 1]])))

    return pose


def set_pose(sim, handle, pose):
    """
    Set the pose of an object in the simulation.
    Receives the pose as a homogeneous transformation matrix and transform
    it to a list of 7 elements (x, y, z, qx, qy, qz, qw).
    """
    pose = pose[:3, :]  # 3 x 4 matrix (12 elements)
    pose = pose.flatten()
    pose = sim.matrixToPose(pose)  # Needs 12 elements
    sim.setObjectPose(handle, sim.handle_world, pose)  # pose as 12 elements
