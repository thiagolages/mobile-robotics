import numpy as np
import cv2
import matplotlib.pyplot as plt
import networkx as nx
from PathPlanner import PathPlanner

class Roadmap(PathPlanner):
    def __init__(self, sim, robot_name, map_name, map_size, map_dims, cell_size=0.25, plot=False):
        super().__init__(sim, robot_name)
        self.map_name = map_name
        self.map_size = map_size
        self.map_dims = map_dims
        self.plot = plot

        # Dimensões do mapa informado em metros (X, Y)
        self.map_size = np.array(map_size)
        self.map_dims = np.array(map_dims)

        # Tamanho da célula do nosso Grid (em metros)
        self.cell_size = cell_size

        self.rows, self.cols = (self.map_dims / self.cell_size).astype(int)
        self.grid = np.zeros((self.rows, self.cols))

    def setup(self):
        self.load_map()
        self.create_graph()
        self.create_graph_8_connected()
        # self.find_path()

    def load_map(self):
        
        self.fig = plt.figure(figsize=(8,8), dpi=100)
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.map = cv2.imread(f"../maps/{self.map_name}.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, self.map_size)
        # Transform to binary map
        self.map[self.map > 127] = 255
        self.map[self.map <= 127] = 0
        # Invert map to better visualize obstacles
        # self.map = 255 - self.map
        self.map = self.map.astype(np.uint8)
        print(f"Roadmap: map_size: {self.map.shape}")

        # Pixels per meter in y and x
        self.ppm_y, self.ppm_x = self.map.shape / self.map_dims

        # Preenchendo o Grid
        # Cada célula recebe o somatório dos valores dos Pixels
        for r in range(self.rows):
            for c in range(self.cols):
                
                xi = int(c*self.cell_size*self.ppm_x)
                xf = int(xi + self.cell_size*self.ppm_x)
                
                yi = int(r*self.cell_size*self.ppm_y)
                yf = int(yi + self.cell_size*self.ppm_y)
                            
                self.grid[r, c] = np.sum(self.map[yi:yf,xi:xf])       

        self.threshold = 127
        self.grid[self.grid > self.threshold] = 255
        self.grid[self.grid<= self.threshold] = 0

        # Add an extra layer of obstacles ("inflation") around existing obstacles in the grid
        # We'll use a value of 200 to represent the inflated layer (between 127 and 255 for a dark reddish color)
        inflated_grid = self.grid.copy()
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r, c] == 255:
                    # For each neighbor (8-connected)
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            nr, nc = r + dr, c + dc
                            if (dr == 0 and dc == 0):
                                continue
                            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                                if inflated_grid[nr, nc] == 0:
                                    inflated_grid[nr, nc] = 200  # Mark as inflated layer

        # Plot the inflated layer on top of the grid, using a dark reddish color
        # We'll create a mask for the inflated cells
        inflated_mask = (inflated_grid == 200)
        # To plot only the inflated layer, create an array with 0s and 1s where the inflated layer is
        inflated_layer = np.zeros_like(self.grid)
        inflated_layer[inflated_mask] = 1
        # Overlay: use a dark red colormap, alpha=0.7 for visibility
        self.ax.imshow(
            inflated_layer,
            cmap=plt.cm.Reds,
            extent=(0, self.map_dims[1], 0, self.map_dims[0]),
            alpha=0.7,
            vmin=0,
            vmax=1
        )
        

        # Plotando Mapa e Células - Fix extents to use map_dims (world coordinates)
        self.ax.imshow(self.map, cmap='Greys', extent=(0, self.map_dims[1], 0, self.map_dims[0]), origin='upper')
        # plt.show(block=True)  # Pause execution until the plot window is closed
        self.ax.imshow(self.grid, cmap='Reds', extent=(0, self.map_dims[1], 0, self.map_dims[0]), alpha=.6)
        # plt.show(block=True)  # Pause execution until the plot window is closed

        # Plotando as linhas do grid para facilitar a visualização
        self.ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
        self.ax.set_xticks(np.arange(0, self.map_dims[1] + self.cell_size, self.cell_size))
        self.ax.set_yticks(np.arange(0, self.map_dims[0] + self.cell_size, self.cell_size))
        self.ax.set_xlim(0, self.map_dims[1])
        self.ax.set_ylim(0, self.map_dims[0])
        self.fig.savefig(f"../plots/{self.map_name}_grid.png")
        # print("showing grid")
        # plt.show()

    def create_graph(self):
        print("creating graph")
        # Criando o Grafo para o nosso Grid
        # Criando vértices em todas as células
        self.G = nx.grid_2d_graph(self.rows, self.cols) 

        # Removendo células que estão em células marcas com obstáculos
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == 255:  
                    self.G.remove_node((r,c))


        self.fig = plt.figure(figsize=(8,8), dpi=100)
        self.ax = self.fig.add_subplot(111, aspect='equal')

        # Grid - Fix extents to use map_dims (world coordinates)
        self.ax.imshow(self.grid, cmap='Greys', extent=(0, self.map_dims[1], 0, self.map_dims[0]))

        self.ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
        self.ax.set_xticks(np.arange(0, self.map_dims[1] + self.cell_size, self.cell_size))
        self.ax.set_yticks(np.arange(0, self.map_dims[0] + self.cell_size, self.cell_size))
        self.ax.set_xlim(0, self.map_dims[1])
        self.ax.set_ylim(0, self.map_dims[0])

        # Os vértices serão plotados no centro da célula - Fix coordinate system
        # node[0] is row (y), node[1] is col (x)
        # Convert from grid coordinates to world coordinates
        self.pos = {node: (node[1] * self.cell_size + self.cell_size/2, 
                          (self.rows - 1 - node[0]) * self.cell_size + self.cell_size/2) 
                   for node in self.G.nodes()}
        nx.draw(self.G, self.pos, font_size=3, with_labels=True, node_size=50, node_color="g", ax=self.ax)
        self.ax.set_title("Grafo 4-conectado do Grid")
        self.fig.savefig(f"../plots/{self.map_name}_4_connected.png")
        plt.show()

    def create_graph_8_connected(self):
        """
        Creates a 2D graph where each node is connected to its 8 closest neighbors.
        This includes horizontal, vertical, and diagonal connections.
        """
        print("creating graph 8 connected")
        # Initialize empty graph
        self.G8 = nx.Graph()
        
        # Add all valid nodes (non-obstacle cells)
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] != 255:  # If not an obstacle
                    self.G8.add_node((r, c))
        
        # Connect each node to its 8 neighbors
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] != 255:  # If not an obstacle
                    # 8 directions: horizontal, vertical and diagonals
                    directions = [
                        (-1, -1), (-1, 0), (-1, 1),  # row above
                        (0, -1),           (0, 1),   # same row
                        (1, -1),  (1, 0),  (1, 1)    # row below
                    ]
                    
                    for dr, dc in directions:
                        nr, nc = r + dr, c + dc
                        # Check if within bounds and not an obstacle
                        if (0 <= nr < self.rows and 
                            0 <= nc < self.cols and 
                            self.grid[nr][nc] != 255):
                            self.G8.add_edge((r, c), (nr, nc))
    
        self.fig = plt.figure(figsize=(8,8), dpi=100)
        self.ax = self.fig.add_subplot(111, aspect='equal')

        # Grid - Fix extents to use map_dims (world coordinates)
        self.ax.imshow(self.grid, cmap='Greys', extent=(0, self.map_dims[1], 0, self.map_dims[0]))
        self.ax.imshow(self.grid, cmap='Reds', extent=(0, self.map_dims[1], 0, self.map_dims[0]), alpha=.6)

        self.ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
        self.ax.set_xticks(np.arange(0, self.map_dims[1] + self.cell_size, self.cell_size))
        self.ax.set_yticks(np.arange(0, self.map_dims[0] + self.cell_size, self.cell_size))
        self.ax.set_xlim(0, self.map_dims[1])
        self.ax.set_ylim(0, self.map_dims[0])

        # Os vértices serão plotados no centro da célula - Fix coordinate system
        # node[0] is row (y), node[1] is col (x)
        # Convert from grid coordinates to world coordinates
        self.pos = {node: (node[1] * self.cell_size + self.cell_size/2, 
                          (self.rows - 1 - node[0]) * self.cell_size + self.cell_size/2) 
                   for node in self.G8.nodes()}
        nx.draw(self.G8, self.pos, font_size=3, with_labels=True, node_size=50, node_color="g", ax=self.ax)
        self.ax.set_title("Grafo 8-conectado do Grid")
        self.fig.savefig(f"../plots/{self.map_name}_8_connected.png")
        plt.show()

    def graph_node_to_world(self, grid_pos):
        """Convert grid coordinates (col, row) to world coordinates (x, y)"""
        np.set_printoptions(precision=2, suppress=True)
        col, row = grid_pos
        map_width, map_height = self.map_dims
        # Convert from cell coordinates to world coordinates in meters
        x = col * self.cell_size - map_width / 2
        y = -row * self.cell_size + map_height / 2
        print("Converted {} to world coordinates: {}".format(grid_pos, (x, y)))
        
        return np.array([x, y])


    def run(self, start=(4, 4), goal=(11, 11), use_Astar=False, use_8_connected=True):
        # Finalmente podemos determinar o menor caminho entre duas células
        # ATENÇÃO para o Sistema de Coordenadas -- relação Índice do grid e Posição no mundo

        if use_8_connected:
            G = self.G8
        else:
            G = self.G
        
        # Convert to top left origin (img coordinates)
        start_img_coord = (start[0] + self.map_dims[1] / 2, -start[1] + self.map_dims[0] / 2)
        goal_img_coord = (goal[0] + self.map_dims[1] / 2, -goal[1] + self.map_dims[0] / 2)

        print(f"start: {start}, goal: {goal}")
        print(f"start_img_coord: {start_img_coord}, goal_img_coord: {goal_img_coord}")

        # Convert from world coordinates to grid coordinates
        start_node = (int(start_img_coord[0] / self.cell_size), int(start_img_coord[1] / self.cell_size))
        end_node = (int(goal_img_coord[0] / self.cell_size), int(goal_img_coord[1] / self.cell_size))        

        print(f"start_node: {start_node}, end_node: {end_node}")

        self.fig = plt.figure(figsize=(8,8), dpi=100)
        self.ax = self.fig.add_subplot(111, aspect='equal')

        # Mapa - Fix extents to use map_dims (world coordinates)
        self.ax.imshow(self.grid, cmap='Greys', extent=(0, self.map_dims[1], 0, self.map_dims[0]))

        # Set proper axis limits and grid
        self.ax.grid(which='major', axis='both', linestyle='-', color='r', linewidth=1)
        self.ax.set_xticks(np.arange(0, self.map_dims[1] + self.cell_size, self.cell_size))
        self.ax.set_yticks(np.arange(0, self.map_dims[0] + self.cell_size, self.cell_size))
        self.ax.set_xlim(0, self.map_dims[1])
        self.ax.set_ylim(0, self.map_dims[0])

        # Caminho
        if use_Astar:
            algorithm = "A*"
            # Using A* with heuristic (if you have a good heuristic function)
            def heuristic(node1, node2):
                # Manhattan distance or Euclidean distance
                return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])
            self.path = nx.astar_path(G, source=start_node, target=end_node, heuristic=heuristic)
        else:
            algorithm = "Dijkstra"
            self.path = nx.shortest_path(G, source=start_node, target=end_node) # Dijkstra's algorithm
        
        # Use the same position calculation as in create_graph methods
        pos = {node: (node[1] * self.cell_size + self.cell_size/2, 
                     (self.rows - 1 - node[0]) * self.cell_size + self.cell_size/2) 
               for node in G.nodes()}
            
        print(f"G.nodes = {G.nodes()}")
        print(f"path before conversion: {self.path}")
        
        nx.draw_networkx_nodes(G, pos, nodelist=self.path, node_size=80, node_color='b')
        self.ax.set_title(f"Caminho encontrado pelo algoritmo {algorithm}, use_8_connected: {use_8_connected}")
        self.fig.savefig(f"../plots/{self.map_name}_path_{start}_to_{goal}_{algorithm}_{'8' if use_8_connected else '4'}-connected.png")
        plt.show()
        
        # Invert path to match the image coordinate system
        self.path = self.path[::-1]
        print(f"path after inversion (img coordinates): {self.path}")

        # Invert X and Y coordinates
        self.path = [(path_point[1], path_point[0]) for path_point in self.path]
        print(f"path after inversion (world coordinates): {self.path}")


        # Convert path to world coordinates
        self.path = [self.graph_node_to_world(path_point) for path_point in self.path]

        # Find a better solution for the path
        
        def simplify_path_linear(path):
            """
            Simplifies a path by reducing stretches of collinear (horizontal/vertical) segments to endpoints.
            This assumes the path is a list of 2D points (x, y).
            """
            if len(path) <= 2:
                return path.copy()

            simplified = [path[0]]
            prev_point = path[0]
            prev_direction = None

            for i in range(1, len(path)):
                curr_point = path[i]
                dx = curr_point[0] - prev_point[0]
                dy = curr_point[1] - prev_point[1]

                # Determine direction: horizontal, vertical, or diagonal (if any)
                if dx == 0 and dy != 0:
                    curr_direction = 'v'
                elif dy == 0 and dx != 0:
                    curr_direction = 'h'
                else:
                    curr_direction = 'd'  # diagonal or other (should be rare on typical grid)

                if prev_direction is None:
                    prev_direction = curr_direction

                if curr_direction != prev_direction:
                    # Direction changed, keep last point
                    simplified.append(prev_point)
                    prev_direction = curr_direction

                prev_point = curr_point

            # Always add the last point
            simplified.append(path[-1])
            return simplified

        simplified_path = simplify_path_linear(self.path)
        print(f"Simplified path: {simplified_path}")

        return self.path, simplified_path
        