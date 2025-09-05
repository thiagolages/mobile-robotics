import time
from pprint import pprint

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

# Configure matplotlib for interactive, non-blocking plots
matplotlib.use("TkAgg")  # Use TkAgg backend for interactive display
plt.ion()  # Enable interactive mode


def build_object_dict(sim, excluded_objects, verbose=False):
    print("Building object dictionary")
    obj_handles = get_frame_handles(sim, excluded_objects, verbose=verbose)
    obj_names = get_frame_names(sim, obj_handles, verbose=verbose)
    obj_poses = get_frame_poses(sim, obj_handles, verbose=verbose)
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
    for handle in handles:
        print(f"Handle: {handle} | Alias: {sim.getObjectAlias(handle)}")
    print(f"Found {len(handles)} handles: {handles}")

    return handles


def get_frame_names(sim, handles, verbose=False):
    names = []
    for handle in handles:
        name = sim.getObjectAlias(handle)
        names.append(name)
        if verbose:
            print(f"Getting name for handle = {handle}")
            print(f"Name: {name}")

    return names


def get_frame_poses(sim, handles, verbose=False):
    poses = []
    for handle in handles:
        pose = sim.getObjectPose(handle)
        poses.append(pose)
        if verbose:
            print(f"Getting pose for handle = {handle}")
            print(f"Pose: {pose}")

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
            "Transform from {} to {}:\n{}".format(
                handle_name if inv else "world",
                "world" if inv else handle_name,
                tf,
            )
        )

    return tf


def handle_sim_start(sim):

    # Do nothing if stepping
    # if stepping:
    #     sim.stopSimulation()
    #     return

    state = sim.getSimulationState()

    if state == sim.simulation_stopped:
        print("-" * 40)
        print("Starting simulation")
        print("-" * 40)
        sim.startSimulation()
    elif state == sim.simulation_paused:
        print("-" * 40)
        print("Simulation is paused, stopping it and then starting it")
        print("-" * 40)
        sim.stopSimulation()
        time.sleep(1)
        print("-" * 40)
        print("Starting simulation")
        print("-" * 40)
        sim.startSimulation()
    else:
        sim.stopSimulation()
        time.sleep(1)
        print("-" * 40)
        print("Starting simulation")
        print("-" * 40)
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
    ax, robot_to_obj_tfs, robot_name, T_w_robot, line_color="orange", line_alpha=0.7, verbose=False
):
    """
    Plot arrows from robot to each object pose.

    Args:
        ax: matplotlib 3D axis
        robot_to_obj_tfs: dictionary of robot-to-object transforms
        robot_name: name of the robot (to skip robot frames)
        line_color: color of the arrows
        line_alpha: transparency of the arrows
        verbose: whether to print verbose output
    """
    robot_origin = T_w_robot[:3, 3]

    for name, T_robot_obj in robot_to_obj_tfs.items():
        if robot_name in name:
            continue  # Skip robot frames

        # Transform object to world frame
        T_w_obj = T_w_robot @ T_robot_obj

        # Get object position relative to robot
        obj_position = T_w_obj[:3, 3]

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
        if verbose:
            print(f"Plotting transform from robot to {name}: \n{T_robot_obj}")
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
    robot_name, 
    robot_to_obj_tfs, 
    T_w_robot=None,
    save_path=None, 
    camera_angle=None, 
    verbose=False, 
    title=None,
    show=True
):
    """
    Plot 3D frames showing the robot-to-object transforms.
    Robot frame is at origin (0,0,0) with identity orientation.

    Args:
        robot_name: name of the robot
        robot_to_obj_tfs: dictionary of robot-to-object transforms
        save_path: path to save the main plot (default: "robot_frames_3d.png")
        camera_angle: tuple (elevation, azimuth) for specific camera angle
        verbose: whether to print verbose output
    """
    if save_path is None:
        save_path = "robot_frames_3d.png"

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")
    frame_size = 0.5

    # Plot robot frame at origin with identity orientation
    if T_w_robot is None:
        T_w_robot = np.eye(4)  # Identity matrix - robot at origin

    plot_frame(ax, T_w_robot, robot_name, frame_size=frame_size)

    # Plot frames for each object relative to robot
    for name, T_robot_obj in robot_to_obj_tfs.items():
        if robot_name in name:
            print(f"Skipping robot frame for {name}")
            continue
        if verbose:
            print(f"Plotting transform from robot to {name}: \n{T_robot_obj}")
            print("-" * 40)
        # Plot obj in world frame
        T_w_obj = T_w_robot @ T_robot_obj
        plot_frame(ax, T_w_obj, name, frame_size=frame_size)

    # Plot lines from robot to objects
    plot_robot_to_object_lines(
        ax, 
        robot_to_obj_tfs, 
        robot_name, 
        T_w_robot,
        verbose=verbose
    )

    # Set up the plot
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if title is None:
        title = "Robot-to-Object Transforms"

    ax.set_title(title)

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

    # Set camera angle if specified
    if camera_angle:
        elev, azim, roll = camera_angle
        ax.view_init(elev=elev, azim=azim, roll=roll)
        if verbose:
            print(f"Set camera view to elev={elev}°, azim={azim}°, roll={roll}°")

    # Save the plot
    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    print(f"3D frame plot saved as '{save_path}'")

    # Force the plot to display immediately without blocking
    if show:
        plt.draw()
        plt.pause(0.01)  # Small pause to ensure the plot window appears
    
    return fig, ax  # Return figure and axes for potential reuse


def generate_random_tf(
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
    if isinstance(pose, list):
        if len(pose) == 12:
            pose = np.array(pose).reshape(3, 4)
            pose = np.vstack((pose, np.array([[0, 0, 0, 1]])))
        else:
            raise ValueError(f"Pose must be a 3x4 matrix or list of 12 elements. Got {len(pose)} elements")
    if isinstance(pose, np.ndarray):
        pass
    else:
        raise ValueError(f"Pose must be a 3x4 matrix or list of 12 elements. Got {type(pose)}")
    
    if pose.shape == (4, 4):
        pose = pose[:3, :]  # 3 x 4 matrix (12 elements)
    if pose.shape == (3, 4):
        pass
    else:
        raise ValueError(f"Pose must be a 3x4 matrix or list of 12 elements. Got {pose.shape} elements")
    
    pose = pose.flatten()
    pose = sim.matrixToPose(pose)  # Needs 12 elements
    sim.setObjectPose(handle, sim.handle_world, pose)  # pose as 12 elements


def plot_sensor_data(
    sensor_data, 
    ax=None, 
    show=False, 
    block=False, 
    robot_path=None, 
    save_path=None, 
    range_dict=None):
    """
    Plot the sensor data. Expects sensor_data as a 4xN numpy array,
    where the first 3 rows are X, Y, Z coordinates, and the 4th row is ignored.
    If ax is provided, plot on the same axes for incremental plotting.
    If show is True, display the plot.
    Returns the matplotlib axes object for reuse.
    Adjusts the Z axis range to be between 0 and 1.
    Sets the camera to look from the top (down the Z axis), with X and Y along the figure width and height.
    Also plots the robot path as a dark dashed line if robot_path is provided.
    """

    if range_dict is None:
        range_dict = {
            "x": [-3, 3],
            "y": [-3, 3],
            "z": [0, 1],
        }

    # If no axes provided, create new figure and axes
    if ax is None:
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Sensor Data Points")
    else:
        fig = ax.get_figure()

    # Plot the robot path as a dark dashed line if provided
    if robot_path is not None and len(robot_path) > 1:
        robot_path_np = np.array(robot_path)
        if len(robot_path_np.shape) == 1:
            robot_path_np = np.array([robot_path_np])

        print(f"Plotting robot path with {len(robot_path)} points")
        if robot_path_np.shape[1] == 3:
            ax.plot(
                robot_path_np[:, 0],
                robot_path_np[:, 1],
                robot_path_np[:, 2],
                color="black",
                linestyle="--",
                linewidth=2,
                label="Robot Path",
                marker='o',
                markersize=3
            )
            # Add start and end markers
            ax.scatter(robot_path_np[0, 0], robot_path_np[0, 1], robot_path_np[0, 2], 
                    color="green", s=100, marker="^", label="Start", zorder=5)
            if robot_path_np.shape[0] > 1:
                ax.scatter(robot_path_np[-1, 0], robot_path_np[-1, 1], robot_path_np[-1, 2], 
                        color="red", s=100, marker="v", label="End", zorder=5)
        else:
            print(f"Warning: Robot path has incorrect shape {robot_path_np.shape}, expected Nx3")

    # Ensure sensor_data is a numpy array
    if sensor_data is not None:
        sensor_data = np.asarray(sensor_data)
        print(f"Plotting sensor data with shape: {sensor_data.shape}")

        # Check shape: should be 4 x N
        if sensor_data.shape[0] != 4:
            raise ValueError(
                f"sensor_data must have shape 4xN, got {sensor_data.shape}"
            )

        # Extract X, Y, Z coordinates
        X = sensor_data[0, :]
        Y = sensor_data[1, :]
        Z = sensor_data[2, :]
        # x_range = [X.min() - 1, X.max() + 1]
        # y_range = [Y.min() - 1, Y.max() + 1]
        ax.set_xlim(range_dict["x"])
        ax.set_ylim(range_dict["y"])

        # Plot all points at once for efficiency
        ax.scatter(X, Y, Z, marker="o", s=1, c="red", alpha=0.6, label="Laser Points")
        print(f"Plotted {len(X)} sensor points")

    # Adjust Z axis range to be between 0 and 1
    ax.set_zlim(range_dict["z"])

    # Set the camera to look from the top (down the Z axis)
    # In matplotlib, elev=90, azim=-90 gives a top-down view with X to right, Y up
    ax.view_init(elev=90, azim=-90)
    
    # Add legend
    ax.legend()

    if save_path is not None:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Sensor data plot saved as '{save_path}'")

    if show:
        plt.draw()
        plt.pause(0.1)  # Longer pause to ensure the plot window appears and updates
        print("Sensor data plot displayed")

    return ax, fig

def visualize_sensor_data(self, sensor_data_list, frame='world'):
    """
    Visualize sensor data points in a 3D plot.
    
    Args:
        sensor_data_list: List of sensor data dictionaries
        frame: Which frame to plot ('laser', 'robot', or 'world')
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    if not sensor_data_list:
        print("No sensor data to visualize")
        return
        
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    points = []
    for data in sensor_data_list:
        if isinstance(data, dict) and 'data' in data:
            # Full scan data
            point = data['data'][f'{frame}_frame']
        else:
            # Single reading data
            point = data[f'{frame}_frame']
        points.append(point)
    
    if points:
        points = np.array(points)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                    c='red', s=50, alpha=0.7, label='Sensor Points')
        
        ax.set_xlabel(f'X ({frame} frame)')
        ax.set_ylabel(f'Y ({frame} frame)')
        ax.set_zlabel(f'Z ({frame} frame)')
        ax.set_title(f'Hokuyo Sensor Points in {frame.title()} Frame')
        ax.legend()
        
        # Set equal aspect ratio
        max_range = np.array([points[:,0].max()-points[:,0].min(),
                            points[:,1].max()-points[:,1].min(),
                            points[:,2].max()-points[:,2].min()]).max() / 2.0
        mid_x = (points[:,0].max()+points[:,0].min()) * 0.5
        mid_y = (points[:,1].max()+points[:,1].min()) * 0.5
        mid_z = (points[:,2].max()+points[:,2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()
    else:
        print("No valid points to plot")