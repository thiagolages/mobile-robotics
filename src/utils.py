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


def plot_robot_to_object_tfs(robot_name, robot_to_obj_tfs):
    """
    Plot 3D frames showing the robot-to-object transforms.
    Robot frame is at origin (0,0,0) with identity orientation.
    """
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
        print(f"Plotting transform from robot to {name}: {T_robot_obj}")
        plot_frame(ax, T_robot_obj, name, frame_size=frame_size)
        print("-" * 40)

    # Set up the plot
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot-to-Object Transforms")

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
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()

    # Save the plot as an image since we're in headless mode
    plt.savefig("robot_frames_3d.png", dpi=300, bbox_inches="tight")
    print("3D frame plot saved as 'robot_frames_3d.png'")
    plt.show()
    plt.close()
