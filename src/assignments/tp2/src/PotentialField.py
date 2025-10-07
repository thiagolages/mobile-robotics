from PathPlanner import PathPlanner
import numpy as np
import cv2
import matplotlib.pyplot as plt
from HokuyoSensorSim import HokuyoSensorSim

class PotentialField(PathPlanner):
    def __init__(self, sim, robot_name, map_name, map_size, goal_pos, 
                 repulsive_distance=1.0, attractive_gain=1.0, repulsive_gain=1.0, 
                 plot=False):
        super().__init__(sim, robot_name)
        self.map_name = map_name
        self.map_size = map_size
        self.goal_pos = goal_pos  # Goal position in world coordinates
        self.repulsive_distance = repulsive_distance  # Distance D for repulsive field
        self.attractive_gain = attractive_gain  # Gain for attractive field
        self.repulsive_gain = repulsive_gain  # Gain for repulsive field
        self.plot = plot
        
        # Map dimensions in world coordinates (meters)
        self.map_dims = np.array([10.0, 10.0])  # Default 10x10 meters
        
        # Initialize Hokuyo sensor for obstacle detection
        self.hokuyo_sensor = None
        try:
            hokuyo_name = f"/{robot_name}/fastHokuyo"
            self.hokuyo_sensor = HokuyoSensorSim(sim, hokuyo_name, is_range_data=True)
            print(f"Hokuyo sensor initialized: {hokuyo_name}")
        except Exception as e:
            print(f"Warning: Could not initialize Hokuyo sensor: {e}")
            print("Repulsive forces will be calculated from map data only")
        
        print(f"PotentialField: map_name: {self.map_name}")
        print(f"Goal position: {self.goal_pos}")
        print(f"Repulsive distance: {self.repulsive_distance}")

    def setup(self):
        self.load_map()
        # self.build_potential_field()
    
    def load_map(self):
        """Load and process the map image"""
        self.map = cv2.imread(f"../maps/{self.map_name}.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, self.map_size)
        
        # Transform to binary map: obstacles = 0 (black), free space = 255 (white)
        self.map[self.map > 0] = 255

        # Invert the map
        self.map = 255 - self.map

        self.map = self.map.astype(np.uint8)
        
        print(f"PotentialField: map_size: {self.map.shape}")
        
        # Calculate pixel to world coordinate conversion
        self.pixel_to_world_scale = self.map_dims / np.array(self.map.shape)
        
        if self.plot:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8), dpi=100)
            ax.imshow(self.map, cmap='gray', origin='upper')
            ax.set_title('Map (Black=Obstacles, White=Free Space)')
            ax.axis('off')
            plt.show()

    def world_to_pixel(self, world_pos):
        """Convert world coordinates to pixel coordinates"""
        pixel_pos = world_pos / self.pixel_to_world_scale
        return pixel_pos.astype(int)

    def pixel_to_world(self, pixel_pos):
        """Convert pixel coordinates to world coordinates"""
        # Since the world (0,0) is at the center, we have to adjust
        pixel_pos[0] -= int(self.map_size[1]/2) # subtract num of columns/2
        pixel_pos[1] += int(self.map_size[0]/2) # add num of rows/2
        return pixel_pos * self.pixel_to_world_scale

    def get_field_vector(self, x, y):
        """Get the potential field vector at world coordinates (x, y)"""
        pixel_pos = self.world_to_pixel(np.array([x, y]))
        if (pixel_pos[0] >= 0 and pixel_pos[0] < self.map.shape[0] and 
            pixel_pos[1] >= 0 and pixel_pos[1] < self.map.shape[1]):
            return self.potential_field[pixel_pos[0], pixel_pos[1]]
        else:
            return np.array([0.0, 0.0])

    def build_potential_field(self):
        """Build the complete potential field combining attractive and repulsive forces"""
        import time
        start_time = time.time()
        
        print("Building potential field...")
        
        # Initialize potential field to zero
        self.potential_field = np.zeros((self.map.shape[0], self.map.shape[1], 2), dtype=np.float32)
        
        # Build attractive field
        self.build_attractive_field()
        
        # Build repulsive field
        # self.build_repulsive_field()
        
        # Normalize the field to prevent extreme values
        self.normalize_field()
        
        end_time = time.time()
        print(f"Potential field built in {end_time - start_time:.2f} seconds")
        
        # Show the complete field
        if self.plot:
            self.show_potential_field()

    def normalize_field(self):
        """Normalize the potential field to prevent extreme values"""
        print("Normalizing field...")
        
        # Calculate field magnitude
        field_magnitude = np.sqrt(self.potential_field[:, :, 0]**2 + self.potential_field[:, :, 1]**2)
        
        # Find maximum magnitude in free space only
        free_space_mask = (self.map > 0)
        if np.any(free_space_mask):
            max_magnitude = np.max(field_magnitude[free_space_mask])
            
            if max_magnitude > 0:
                # Normalize to prevent extreme values
                normalization_factor = min(1.0, 10.0 / max_magnitude)  # Cap at reasonable values
                self.potential_field *= normalization_factor
                print(f"Field normalized with factor: {normalization_factor:.3f}")
            else:
                print("Warning: No field magnitude found in free space")
        else:
            print("Warning: No free space found in map")

    def build_attractive_field(self):
        """Build attractive field towards the goal"""
        print("Building attractive field...")
        
        goal_pixel = self.world_to_pixel(self.goal_pos)
        
        step = max(1, self.map.shape[0] // 40)  # Sample every 40th pixel for better visibility

        # Cover the whole map using the same pixel interval logic
        min_i = 0
        max_i = self.map.shape[0]
        min_j = 0
        max_j = self.map.shape[1]
        
        for i in range(min_i, max_i, step):
            for j in range(min_j, max_j, step):
                # Skip obstacle pixels
                if self.map[i, j] == 0:
                    continue
                    
                # Current position in world coordinates
                current_world = self.pixel_to_world(np.array([i, j]))
                print(f"Pixel to world ({i},{j})= {self.pixel_to_world(np.array([i, j]))}")
                
                # Vector from current position to goal
                to_goal = self.goal_pos - current_world
                distance = np.linalg.norm(to_goal)
                
                if distance > 0:
                    # Attractive force: proportional to distance (linear potential)
                    attractive_force = self.attractive_gain * to_goal
                    print(f"Attractive force calculated for pixel ({i},{j}): {attractive_force}")
                    self.potential_field[i, j] += attractive_force

    def build_repulsive_field(self):
        """Build repulsive field from obstacles"""
        print("Building repulsive field...")
        
        # Find all obstacle pixels
        obstacle_pixels = np.array(np.where(self.map == 0)).T
        
        print(f"Found {len(obstacle_pixels)} obstacle pixels")
        
        # Convert repulsive distance to pixels
        repulsive_distance_pixels = int(self.repulsive_distance / self.pixel_to_world_scale[0])
        
        print(f"Repulsive distance: {self.repulsive_distance:.2f}m = {repulsive_distance_pixels} pixels")
        
        # For very large maps, sample obstacle pixels to avoid excessive computation
        max_obstacles = 1000  # Maximum number of obstacles to process
        if len(obstacle_pixels) > max_obstacles:
            # Sample obstacles uniformly
            step = len(obstacle_pixels) // max_obstacles
            obstacle_pixels = obstacle_pixels[::step]
            print(f"Sampled {len(obstacle_pixels)} obstacles for efficiency")
        
        for obs_i, obs_j in obstacle_pixels:
            # Current obstacle position in world coordinates
            obs_world = self.pixel_to_world(np.array([obs_i, obs_j]))
            
            # Only check pixels within repulsive distance of this obstacle
            min_i = max(0, obs_i - repulsive_distance_pixels)
            max_i = min(self.map.shape[0], obs_i + repulsive_distance_pixels + 1)
            min_j = max(0, obs_j - repulsive_distance_pixels)
            max_j = min(self.map.shape[1], obs_j + repulsive_distance_pixels + 1)
            
            for i in range(min_i, max_i):
                for j in range(min_j, max_j):
                    # Skip obstacle pixels
                    if self.map[i, j] == 0:
                        continue
                    
                    # Current position in world coordinates
                    current_world = self.pixel_to_world(np.array([i, j]))
                    
                    # Vector from obstacle to current position
                    from_obstacle = current_world - obs_world
                    distance = np.linalg.norm(from_obstacle)
                    
                    # Apply repulsive force if within influence distance
                    if 0 < distance <= self.repulsive_distance:
                        # Repulsive force formula: F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * unit_vector
                        # This creates a very strong force near obstacles that decreases with distance
                        rep_magnitude = self.repulsive_gain * (1/distance - 1/self.repulsive_distance) * (1/distance**2)
                        rep_force = rep_magnitude * (from_obstacle / distance)
                        self.potential_field[i, j] += rep_force

    def show_potential_field(self):
        """Visualize the potential field"""
        print("Showing potential field...")
        # Extract field components
        Fx = self.potential_field[:, :, 0]
        Fy = self.potential_field[:, :, 1]
        print(f"Fx: {Fx}")
        print(f"Fy: {Fy}")
        print(f"Fx.shape: {Fx.shape}")
        print(f"Fy.shape: {Fy.shape}")
        
        # Calculate field magnitude for visualization
        field_magnitude = np.sqrt(Fx**2 + Fy**2)
        
        # Create a mask for free space (non-obstacle pixels)
        free_space_mask = (self.map > 0)
        
        # Set field magnitude to 0 on obstacles for cleaner visualization
        field_magnitude_clean = field_magnitude.copy()
        field_magnitude_clean[~free_space_mask] = 0
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(16, 12), dpi=100)
        
        # Plot 1: Original map
        axes[0, 0].imshow(self.map, cmap='gray', origin='upper')
        axes[0, 0].set_title('Original Map')
        axes[0, 0].axis('off')

        # Plot 2: Field magnitude (only on free space)
        step = max(1, self.map.shape[0] // 40)  # Sample every 40th pixel for better visibility
        X = np.arange(0, self.map.shape[0], step)
        Y = np.arange(0, self.map.shape[1], step)
        XX, YY = np.meshgrid(X, Y, indexing='ij')
        U = Fx[XX, YY]
        V = Fy[XX, YY]
        free_space_mask_sampled = free_space_mask[XX, YY]
        
        # Set vectors to 0 on obstacles
        U[~free_space_mask_sampled] = 0
        V[~free_space_mask_sampled] = 0
        valid_vectors = (U != 0) | (V != 0)

        im = axes[0, 1].imshow(field_magnitude_clean, cmap='hot', origin='upper')
        axes[0, 1].quiver(YY[valid_vectors], XX[valid_vectors], 
                             U[valid_vectors], V[valid_vectors], 
                             color='blue', scale=30, width=0.004, headwidth=3)
        axes[0, 1].set_title('Field Magnitude (Free Space Only)')
        axes[0, 1].axis('off')
        plt.colorbar(im, ax=axes[0, 1])
        
        # Plot 3: Vector field (sampled, only on free space)
        step = max(1, self.map.shape[0] // 40)  # Sample every 40th pixel for better visibility
        X = np.arange(0, self.map.shape[0], step)
        Y = np.arange(0, self.map.shape[1], step)
        XX, YY = np.meshgrid(X, Y, indexing='ij')
        
        # Only show vectors on free space
        U = Fx[XX, YY]
        V = Fy[XX, YY]
        free_space_mask_sampled = free_space_mask[XX, YY]
        
        # Set vectors to 0 on obstacles
        U[~free_space_mask_sampled] = 0
        V[~free_space_mask_sampled] = 0
        
        axes[1, 0].imshow(self.map, cmap='gray', origin='upper', alpha=0.7)
        # Only plot non-zero vectors
        valid_vectors = (U != 0) | (V != 0)
        if np.any(valid_vectors):
            axes[1, 0].quiver(YY[valid_vectors], XX[valid_vectors], 
                             U[valid_vectors], V[valid_vectors], 
                             color='blue', scale=30, width=0.004, headwidth=3)
        axes[1, 0].set_title('Vector Field (Free Space Only)')
        axes[1, 0].axis('off')
        
        # Plot 4: Combined visualization
        axes[1, 1].imshow(field_magnitude_clean, cmap='hot', origin='upper', alpha=0.8)
        
        # Only plot non-zero vectors
        if np.any(valid_vectors):
            axes[1, 1].quiver(YY[valid_vectors], XX[valid_vectors], 
                             U[valid_vectors], V[valid_vectors], 
                             color='white', scale=30, width=0.004, headwidth=3)
        
        # Mark goal position (only one goal)
        goal_pixel = self.world_to_pixel(self.goal_pos)
        axes[1, 1].plot(goal_pixel[1], goal_pixel[0], 'g*', markersize=20, label='Goal', markeredgecolor='black', markeredgewidth=1)
        axes[1, 1].set_title('Field Magnitude + Vectors + Goal')
        axes[1, 1].axis('off')
        axes[1, 1].legend()
        
        plt.tight_layout()
        plt.show()

    def get_repulsive_force(self, robot_pos):
        """Calculate repulsive force from obstacles using sensor data
        
        Args:
            robot_pos: Current robot position [x, y]
            
        Returns:
            tuple: (vx_rep, vy_rep) repulsive force components
        """
        if self.hokuyo_sensor is None:
            # Fallback to map-based repulsive force if sensor not available
            return self._get_map_repulsive_force(robot_pos)
        
        try:
            # Get sensor data (range and angle)
            sensor_data = self.hokuyo_sensor.getSensorData(verbose=False)
            
            if len(sensor_data) == 0:
                return 0.0, 0.0
            
            # Extract angles and ranges
            angles = sensor_data[:, 0]  # First column: angles
            ranges = sensor_data[:, 1]  # Second column: ranges
            
            # Filter out invalid readings (too far or too close)
            valid_mask = (ranges > 0.1) & (ranges < self.repulsive_distance)
            valid_angles = angles[valid_mask]
            valid_ranges = ranges[valid_mask]
            
            if len(valid_ranges) == 0:
                return 0.0, 0.0
            
            # Calculate obstacle positions in world coordinates
            # Convert from robot frame to world frame
            robot_x, robot_y = robot_pos
            
            # Calculate obstacle positions relative to robot
            obs_x = robot_x + valid_ranges * np.cos(valid_angles)
            obs_y = robot_y + valid_ranges * np.sin(valid_angles)
            
            # Calculate repulsive forces for each detected obstacle
            total_force_x = 0.0
            total_force_y = 0.0
            
            for i in range(len(valid_ranges)):
                # Distance from robot to obstacle
                distance = valid_ranges[i]
                
                # Only apply repulsive force if within influence distance
                if distance < self.repulsive_distance:
                    # Unit vector from obstacle to robot
                    dx = robot_x - obs_x[i]
                    dy = robot_y - obs_y[i]
                    norm = np.sqrt(dx**2 + dy**2)
                    
                    if norm > 0:
                        # Repulsive force formula: F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * unit_vector
                        rep_magnitude = self.repulsive_gain * (1/distance - 1/self.repulsive_distance) * (1/distance**2)
                        
                        # Add to total force
                        total_force_x += rep_magnitude * (dx / norm)
                        total_force_y += rep_magnitude * (dy / norm)
            
            # normalize by the number of valid ranges
            total_force_x /= len(valid_ranges)
            total_force_y /= len(valid_ranges)
            
            return total_force_x, total_force_y
            
        except Exception as e:
            print(f"Error calculating sensor-based repulsive force: {e}")
            # Fallback to map-based calculation
            return self._get_map_repulsive_force(robot_pos)
    
    def _get_map_repulsive_force(self, robot_pos):
        """Fallback method to calculate repulsive force from map data
        
        Args:
            robot_pos: Current robot position [x, y]
            
        Returns:
            tuple: (vx_rep, vy_rep) repulsive force components
        """
        # This is a simplified fallback - in practice, you might want to
        # implement a more sophisticated map-based repulsive force calculation
        return 0.0, 0.0

    def run(self):
        """Main execution method"""
        pass