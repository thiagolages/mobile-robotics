import numpy as np
import cv2
import matplotlib.pyplot as plt
from collections import namedtuple
from math import sqrt, atan2
from PathPlanner import PathPlanner


Node = namedtuple("Node", ["x", "y", "parent"])

class RRT(PathPlanner):
    def __init__(self, sim, robot_name, map_name, map_size, map_dims, plot=False, seed=42):
        super().__init__(sim, robot_name)
        self.map_name = map_name
        self.map_size = map_size
        self.plot = plot

        self.map_size = np.array(map_size) # in pixels
        self.map_dims = np.array(map_dims) # in meters

        # Parameters you can tune
        self.step_size = 0.4          # [map units] step when steering toward samples
        self.max_iters = 3000         # max nodes to add
        self.goal_sample_rate = 0.06  # probability to sample the goal
        self.collision_resolution = 0.010  # step used for segment collision checking

        # Internal state
        self.map = None
        self.fig = None
        self.ax = None
        self.tree = []         # list[Node]
        self.edges = []        # list[(NodeIdx, NodeIdx)]

        # Set a seed for reproducibility
        np.random.seed(seed)

    def setup(self):
        self._load_map()
        if self.plot:
            self._setup_plot()

    def _load_map(self):
        # Load and binary-threshold map; free=255 (white), obstacle=0 (black)
        self.map = cv2.imread(f"../maps/{self.map_name}.png", cv2.IMREAD_GRAYSCALE)
        if self.map is None:
            raise FileNotFoundError(f"Map not found: ../maps/{self.map_name}.png")
        self.map = cv2.resize(self.map, self.map_size)
        self.map[self.map > 0] = 255

        # Invert so that obstacles are black and free space is white
        self.map = 255 - self.map

        self.map = self.map.astype(np.uint8)
        # World extents in the same units as the image (top-left origin)
        # self.map_dims = np.array([self.map.shape[0], self.map.shape[1]], dtype=float)

    def _setup_plot(self):
        plt.close('all')
        plt.ion()
        self.fig = plt.figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111, aspect="equal")
        self.ax.imshow(self.map, cmap="gray")#, origin="upper")  # top-left origin
        # self.ax.set_xlim(0, self.map_dims[1])
        # self.ax.set_ylim(self.map_dims[0], 0)  # invert y for image-like coords
        self.ax.set_title("RRT planning")

    def _in_bounds(self, x, y):
        return 0 <= x < self.map_size[1] and 0 <= y < self.map_size[0] # in pixels

    def _is_free(self, x, y):
        if not self._in_bounds(x, y):
            return False
        return self.map[int(y), int(x)] == 255 # in pixels

    def _sample(self, goal_px):
        if np.random.rand() < self.goal_sample_rate:
            return np.array([goal_px[0], goal_px[1]], dtype=float)
        return np.array([
            np.random.uniform(0, self.map_size[0]),
            np.random.uniform(0, self.map_size[1]),
        ], dtype=float)

    def _nearest(self, p):
        # Return index of nearest node in the tree to point p
        coords = np.array([[n.x, n.y] for n in self.tree], dtype=float)
        d2 = np.sum((coords - p) ** 2, axis=1)
        return int(np.argmin(d2))

    def _steer(self, from_node, to_point):
        dx, dy = to_point[0] - from_node.x, to_point[1] - from_node.y
        dist = sqrt(dx * dx + dy * dy)
        if dist < 1e-9:
            return np.array([from_node.x, from_node.y], dtype=float)
        scale = min(self.step_size, dist) / dist
        return np.array([from_node.x + dx * scale, from_node.y + dy * scale], dtype=float)

    def _collision_free(self, p0, p1):
        # Sample along the segment and ensure all points are in free space
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        length = sqrt(dx * dx + dy * dy)
        if length < 1e-9:
            return self._is_free(p0[0], p0[1])
        steps = max(2, int(length / self.collision_resolution))
        for i in range(steps + 1):
            t = i / steps
            x = p0[0] + t * dx
            y = p0[1] + t * dy
            if not self._is_free(x, y):
                return False
        return True

    def _draw_increment(self, parent, new_node, start_px, goal_px):
        if not self.plot:
            return
        # if self.ax is None:
        #     self._setup_plot()
        # Draw tree edge
        self.ax.plot([parent.x, new_node.x], [parent.y, new_node.y], c="tab:blue", lw=1, alpha=0.9)
        # Draw new vertex
        self.ax.plot(new_node.x, new_node.y, "o", ms=3, c="tab:blue", alpha=0.9)
        # Plot start_px/goal and legend only if not already present
        if not hasattr(self, "_plotted_start_goal") or not self._plotted_start_goal:
            self.ax.plot(start_px[0], start_px[1], "o", ms=7, c="tab:green", label="start")
            self.ax.plot(goal_px[0], goal_px[1], "o", ms=7, c="tab:red", label="goal")
            self.ax.legend(loc="upper right", fontsize=8, frameon=True)
            self._plotted_start_goal = True
        self.ax.legend(loc="upper right", fontsize=8, frameon=True)
        from IPython.display import display, clear_output
        clear_output(wait=True)
        display(self.fig)
        plt.pause(0.001)

    def _draw_path(self, path):
        if not self.plot or self.ax is None:
            return
        xs = [n.x for n in path]
        ys = [n.y for n in path]
        self.ax.plot(xs, ys, c="gold", lw=3, alpha=0.95)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _reconstruct_path(self, goal_idx):
        path = []
        idx = goal_idx
        while idx is not None:
            node = self.tree[idx]
            path.append(node)
            idx = node.parent
        path.reverse()
        return path

    def _reached_goal(self, node, goal, tol=1.0):
        print(f"RRT: node: {node.x}, {node.y}, goal: {goal[0]}, {goal[1]}, tol: {tol}")
        print("distance: ", sqrt((node.x - goal[0])**2 + (node.y - goal[1])**2))
        return sqrt((node.x - goal[0])**2 + (node.y - goal[1])**2) <= tol

    def run(self, 
        start, 
        goal, 
        step_size=None, 
        max_iters=None, 
        goal_sample_rate=None, 
        animate=True,
        collision_resolution=None,
    ):
        # start, goal are (x, y) in image/world coordinates matching map indexing
        if self.map is None:
            self.setup()

        if step_size is not None:
            self.step_size = float(step_size)
        if max_iters is not None:
            self.max_iters = int(max_iters)
        if goal_sample_rate is not None:
            self.goal_sample_rate = float(goal_sample_rate)
        if collision_resolution is not None:
            self.collision_resolution = float(collision_resolution)

        print(f"RRT: start: {start}, goal: {goal}")
        # Convert start and goal from meters (world coordinates) to pixel (image/map) coordinates for _is_free
        # Assumes self.map_dims are (width_m, height_m) in meters, and self.map.shape is in pixels
        # start: (x, y) in meters
        # (0,0) world is at image center
        def world_to_img_coords(point):
            # Map (x, y) meters to image pixel (row, col)
            # x: right (meters), y: up (meters)
            x, y = point
            map_h_pix, map_w_pix = self.map.shape[:2]
            width_m, height_m = self.map_dims

            # Move (0,0) to map center, flip y for image top-left 0 origin
            col = int((x + width_m / 2) * (map_w_pix / width_m))
            row = int((height_m / 2 - y) * (map_h_pix / height_m))
            # Clamp to map bounds
            row = max(0, min(map_h_pix - 1, row))
            col = max(0, min(map_w_pix - 1, col))
            return col, row

        # Convert start and goal to pixel coordinates for _is_free
        start_px = world_to_img_coords(start) # x, y
        goal_px = world_to_img_coords(goal) # x, y
        print(f"RRT: start_px: {start_px}, goal_px: {goal_px}")

        if not self._is_free(start_px[0], start_px[1]):
            raise ValueError("Start is in obstacle or out of bounds")
        if not self._is_free(goal_px[0], goal_px[1]):
            raise ValueError("Goal is in obstacle or out of bounds")

        # Initialize tree
        self.tree = [Node(x=float(start_px[0]), y=float(start_px[1]), parent=None)]
        self.edges = []

        goal_idx = None
        for _ in range(self.max_iters):
            print("--------------------------------")
            print(f"RRT: iter: {_}, goal_idx: {goal_idx}")
            sample = self._sample(goal_px)
            print("Sample: ", sample[0], sample[1])
            nearest_idx = self._nearest(sample)
            print("Nearest idx: ", nearest_idx)
            nearest_node = self.tree[nearest_idx]
            print("Nearest node: ", nearest_node.x, nearest_node.y)
            new_point = self._steer(nearest_node, sample)
            print("New point: ", new_point[0], new_point[1])

            if not self._collision_free(np.array([nearest_node.x, nearest_node.y]), new_point):
                print("Collision detected in pixel coordinates: ", nearest_node.x, nearest_node.y, new_point[0], new_point[1])
                continue

            new_node = Node(x=new_point[0], y=new_point[1], parent=nearest_idx)
            self.tree.append(new_node)
            self.edges.append((nearest_idx, len(self.tree) - 1))

            if animate and self.plot:
                self._draw_increment(nearest_node, new_node, start_px, goal_px)

            if self._reached_goal(new_node, goal_px, tol=1.0):
                goal_idx = len(self.tree) - 1
                print(f"RRT: goal reached")
                break

        if goal_idx is None:
            # Try to connect directly to goal from nearest node at the end
            nearest_idx = self._nearest(np.array(goal_px))
            nearest_node = self.tree[nearest_idx]
            if self._collision_free(np.array([nearest_node.x, nearest_node.y]), np.array(goal_px)):
                goal_node = Node(x=float(goal_px[0]), y=float(goal_px[1]), parent=nearest_idx)
                self.tree.append(goal_node)
                self.edges.append((nearest_idx, len(self.tree) - 1))
                goal_idx = len(self.tree) - 1
                if animate and self.plot:
                    self._draw_increment(nearest_node, goal_node, start_px, goal_px)

        path = []
        if goal_idx is not None:
            path = self._reconstruct_path(goal_idx)
            if animate and self.plot:
                self._draw_path(path)
        
        if animate and self.plot:
            import matplotlib.pyplot as plt
            print("RRT: Visualization -- figure shown (window open, execution continues).")
            plt.ioff()
            # Save the plotted image before showing it
            if self.fig is not None:
                self.fig.savefig("rrt_tree.png", bbox_inches='tight')
                print("RRT: Tree image saved as rrt_tree.png")
            plt.show(block=False)  # Show non-blocking window

        # Transform the path from pixel to world coordinates and return both
        path_px = [[node.x, node.y] for node in path]

        def img_to_world_coords(pixel):
            # Map image pixel (col, row) back to (x, y) meters
            # col: horizontal pixel (x), row: vertical pixel (y, top=0)
            x_px, y_px = pixel
            map_h_pix, map_w_pix = self.map.shape[:2]
            height_m, width_m = self.map_dims

            # Compute x in meters: x = (col / map_w_pix) * width_m - width_m / 2
            x = (x_px / map_w_pix) * width_m - width_m / 2
            # Compute y in meters: y = height_m / 2 - (row / map_h_pix) * height_m
            y = height_m / 2 - (y_px / map_h_pix) * height_m
            return x, y

        # Return in pixel path, world path (both as lists of [x, y])
        return self.tree, self.edges, path_px, [img_to_world_coords(px) for px in path_px]