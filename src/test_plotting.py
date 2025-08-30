import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")  # Use Agg backend for headless operation


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


def test_3d_frame_plotting():
    """
    Test the 3D frame plotting with sample transforms.
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Robot frame at origin with identity orientation
    robot_tf = np.eye(4)
    plot_frame(ax, robot_tf, "Robot", frame_size=1.0)

    # Sample object transforms relative to robot
    sample_transforms = {
        "Object1": np.array(
            [[1, 0, 0, 2], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]]
        ),
        "Object2": np.array(
            [[0, -1, 0, -1], [1, 0, 0, 2], [0, 0, 1, 1], [0, 0, 0, 1]]
        ),
        "Object3": np.array(
            [
                [0.707, -0.707, 0, 0],
                [0.707, 0.707, 0, -2],
                [0, 0, 1, 0.5],
                [0, 0, 0, 1],
            ]
        ),
    }

    # Plot frames for each sample object
    for name, T_robot_obj in sample_transforms.items():
        print(f"Plotting transform from robot to {name}:")
        print(T_robot_obj)
        plot_frame(ax, T_robot_obj, name, frame_size=1.0)
        print("-" * 40)

    # Set up the plot
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot-to-Object Coordinate Frames\n")

    # Set equal aspect ratio and reasonable limits
    max_range = 3.0
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])

    # Make the axes equal
    ax.set_box_aspect([1, 1, 1])

    # Add a grid
    ax.grid(True, alpha=0.3)

    # Add legend
    from matplotlib.lines import Line2D

    legend_elements = [
        Line2D([0], [0], color="red", lw=2, label="X-axis"),
        Line2D([0], [0], color="green", lw=2, label="Y-axis"),
        Line2D([0], [0], color="blue", lw=2, label="Z-axis"),
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()

    # Save the plot as an image since we're in headless mode
    plt.savefig("test_robot_frames_3d.png", dpi=300, bbox_inches="tight")
    print("Test 3D frame plot saved as 'test_robot_frames_3d.png'")
    plt.close()


if __name__ == "__main__":
    test_3d_frame_plotting()
