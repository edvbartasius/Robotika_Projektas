"""
CoppeliaSim Maze Generator using Prim's Algorithm
"""

import random
from collections import deque
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class MazeGenerator:
    def __init__(self, width, height, cell_size=2.0, seed=None):
        """
        Initialize maze generator

        Args:
            width: Number of cells horizontally
            height: Number of cells vertically
            cell_size: Size of each cell in meters (default 2.0m to fit 1.5m robot)
            seed: Random seed for reproducible mazes (optional, None for random)
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.seed = seed
        self.cells = []
        self.walls = []
        self.obstacles = set()  # Track cells with obstacles

        # Initialize grid
        for y in range(height):
            row = []
            for x in range(width):
                row.append({
                    'x': x,
                    'y': y,
                    'visited': False,
                    'walls': {'north': True, 'south': True, 'east': True, 'west': True},
                    'has_obstacle': False
                })
            self.cells.append(row)
    
    def get_cell(self, x, y):
        """Get cell at position (x, y)"""
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.cells[y][x]
        return None
    
    def get_neighbors(self, x, y):
        """Get unvisited neighbors of cell at (x, y)"""
        neighbors = []
        directions = [
            (0, -1, 'north'),  # North
            (0, 1, 'south'),    # South
            (1, 0, 'east'),     # East
            (-1, 0, 'west')     # West
        ]
        
        for dx, dy, direction in directions:
            nx, ny = x + dx, y + dy
            neighbor = self.get_cell(nx, ny)
            if neighbor and not neighbor['visited']:
                neighbors.append((nx, ny, direction))
        
        return neighbors
    
    def get_visited_neighbors(self, x, y):
        """Get visited neighbors of cell at (x, y)"""
        neighbors = []
        directions = [
            (0, -1, 'south'),  # North neighbor (we come from its south)
            (0, 1, 'north'),    # South neighbor (we come from its north)
            (1, 0, 'west'),     # East neighbor (we come from its west)
            (-1, 0, 'east')     # West neighbor (we come from its east)
        ]
        
        for dx, dy, direction in directions:
            nx, ny = x + dx, y + dy
            neighbor = self.get_cell(nx, ny)
            if neighbor and neighbor['visited']:
                neighbors.append((nx, ny, direction))
        
        return neighbors
    
    def remove_wall(self, x1, y1, x2, y2):
        """Remove wall between two adjacent cells"""
        dx = x2 - x1
        dy = y2 - y1
        
        cell1 = self.get_cell(x1, y1)
        cell2 = self.get_cell(x2, y2)
        
        if dx == 1:  # Moving east
            cell1['walls']['east'] = False
            cell2['walls']['west'] = False
        elif dx == -1:  # Moving west
            cell1['walls']['west'] = False
            cell2['walls']['east'] = False
        elif dy == 1:  # Moving south
            cell1['walls']['south'] = False
            cell2['walls']['north'] = False
        elif dy == -1:  # Moving north
            cell1['walls']['north'] = False
            cell2['walls']['south'] = False
    
    def generate_prim(self):
        """Generate maze using randomized Prim's algorithm"""
        # Set random seed if provided for reproducibility
        if self.seed is not None:
            random.seed(self.seed)

        # Start from random cell
        start_x = random.randint(0, self.width - 1)
        start_y = random.randint(0, self.height - 1)
        
        start_cell = self.get_cell(start_x, start_y)
        start_cell['visited'] = True
        
        # Frontier set: cells adjacent to visited cells
        frontier = set()
        
        # Add neighbors of start cell to frontier
        for nx, ny, _ in self.get_neighbors(start_x, start_y):
            frontier.add((nx, ny))
        
        # Main loop
        while frontier:
            # Choose random frontier cell
            fx, fy = random.choice(list(frontier))
            frontier.remove((fx, fy))
            
            frontier_cell = self.get_cell(fx, fy)
            frontier_cell['visited'] = True
            
            # Get visited neighbors
            visited_neighbors = self.get_visited_neighbors(fx, fy)
            
            if visited_neighbors:
                # Connect to random visited neighbor
                nx, ny, direction = random.choice(visited_neighbors)
                self.remove_wall(fx, fy, nx, ny)
            
            # Add unvisited neighbors to frontier
            for nx, ny, _ in self.get_neighbors(fx, fy):
                if (nx, ny) not in frontier:
                    frontier.add((nx, ny))
        
        # Create entrance and exit
        # Entrance at top-left (remove north wall of cell at 0,0)
        entrance_cell = self.get_cell(0, 0)
        entrance_cell['walls']['north'] = False

        # Exit at bottom-right (remove south wall of cell at width-1, height-1)
        exit_cell = self.get_cell(self.width - 1, self.height - 1)
        exit_cell['walls']['south'] = False

        return self.cells

    def get_accessible_neighbors(self, x, y):
        """Get neighbors accessible from cell (x, y) considering walls"""
        neighbors = []
        cell = self.get_cell(x, y)

        if not cell:
            return neighbors

        # Check each direction
        if not cell['walls']['north']:
            neighbors.append((x, y - 1))
        if not cell['walls']['south']:
            neighbors.append((x, y + 1))
        if not cell['walls']['east']:
            neighbors.append((x + 1, y))
        if not cell['walls']['west']:
            neighbors.append((x - 1, y))

        return neighbors

    def find_shortest_path(self, start_x, start_y, end_x, end_y):
        """Find shortest path using BFS from start to end"""
        queue = deque([(start_x, start_y, [])])
        visited = set()
        visited.add((start_x, start_y))

        while queue:
            x, y, path = queue.popleft()
            current_path = path + [(x, y)]

            # Check if we reached the goal
            if x == end_x and y == end_y:
                return current_path

            # Explore neighbors
            for nx, ny in self.get_accessible_neighbors(x, y):
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, current_path))

        return []  # No path found

    def place_obstacles(self, num_obstacles=5):
        """Place obstacles in cells NOT on the shortest path

        Args:
            num_obstacles: Number of obstacles to place
        """
        # Find shortest path from entrance (0, 0) to exit (width-1, height-1)
        shortest_path = self.find_shortest_path(0, 0, self.width - 1, self.height - 1)

        if not shortest_path:
            print("Warning: No path found from entrance to exit!")
            return

        # Convert path to set for fast lookup
        path_cells = set(shortest_path)

        # Get all cells not on the path
        available_cells = []
        for y in range(self.height):
            for x in range(self.width):
                if (x, y) not in path_cells:
                    available_cells.append((x, y))

        # Randomly select cells for obstacles
        if len(available_cells) < num_obstacles:
            print(f"Warning: Only {len(available_cells)} cells available for obstacles (requested {num_obstacles})")
            num_obstacles = len(available_cells)

        if num_obstacles > 0:
            obstacle_cells = random.sample(available_cells, num_obstacles)

            for x, y in obstacle_cells:
                cell = self.get_cell(x, y)
                cell['has_obstacle'] = True
                self.obstacles.add((x, y))

        print(f"Placed {num_obstacles} obstacles")
        print(f"Shortest path length: {len(shortest_path)} cells")

    def create_in_coppeliasim(self, sim):
        """Create the maze in CoppeliaSim"""
        wall_height = 0.3  # 30cm walls
        wall_thickness = 0.05  # 5cm thick walls
        line_width = 0.08  # 8cm wide guide lines
        line_length = 1.0 # Length of line segments from center
        
        # Create floor
        floor_width = self.width * self.cell_size
        floor_height = self.height * self.cell_size
        floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid, 
                                         [floor_width, floor_height, 0.01])
        sim.setObjectPosition(floor, -1, [floor_width/2, floor_height/2, -0.005])
        sim.setObjectColor(floor, 0, sim.colorcomponent_ambient_diffuse, [0.8, 0.8, 0.8])
        sim.setObjectAlias(floor, 'MazeFloor')
        
        # Create walls and lines
        for y in range(self.height):
            for x in range(self.width):
                cell = self.cells[y][x]
                
                # Cell center position in world coordinates
                cx = (x + 0.5) * self.cell_size
                cy = (y + 0.5) * self.cell_size
                
                # Count open passages
                open_count = sum([
                    not cell['walls']['north'],
                    not cell['walls']['south'],
                    not cell['walls']['east'],
                    not cell['walls']['west']
                ])
                
                # Create center square if there are 2+ open passages (L, T, or + shape)
                if open_count >= 2:
                    self._create_guide_line(sim, cx, cy, 
                                          line_width, line_width, 0)
                
                # Create guide lines based on open passages
                # Lines extend from center towards open walls
                if not cell['walls']['north']:
                    # Line going north (towards negative Y)
                    self._create_guide_line(sim, cx, cy - line_length/2, 
                                          line_width, line_length, 0)
                
                if not cell['walls']['south']:
                    # Line going south (towards positive Y)
                    self._create_guide_line(sim, cx, cy + line_length/2, 
                                          line_width, line_length, 0)
                
                if not cell['walls']['east']:
                    # Line going east (towards positive X)
                    self._create_guide_line(sim, cx + line_length/2, cy, 
                                          line_length, line_width, 0)
                
                if not cell['walls']['west']:
                    # Line going west (towards negative X)
                    self._create_guide_line(sim, cx - line_length/2, cy, 
                                          line_length, line_width, 0)
                
                # Create walls with proper dimensions for each direction
                if cell['walls']['north']:
                    # Horizontal wall (runs along X axis)
                    self._create_wall(sim, cx, cy - self.cell_size/2, 
                                    self.cell_size, wall_thickness, wall_height)
                
                if cell['walls']['south']:
                    # Horizontal wall (runs along X axis)
                    self._create_wall(sim, cx, cy + self.cell_size/2, 
                                    self.cell_size, wall_thickness, wall_height)
                
                if cell['walls']['east']:
                    # Vertical wall (runs along Y axis) - swap X and Y dimensions
                    self._create_wall(sim, cx + self.cell_size/2, cy, 
                                    wall_thickness, self.cell_size, wall_height)
                
                if cell['walls']['west']:
                    # Vertical wall (runs along Y axis) - swap X and Y dimensions
                    self._create_wall(sim, cx - self.cell_size/2, cy,
                                    wall_thickness, self.cell_size, wall_height)

                # Create obstacle if this cell has one
                if cell['has_obstacle']:
                    self._create_obstacle(sim, cx, cy)

    def _create_obstacle(self, sim, x, y):
        """Create an obstacle (cylinder) in the cell

        Args:
            x, y: position in world coordinates
        """
        obstacle_height = 0.4  # 40cm tall
        obstacle_radius = 0.25  # 25cm radius (50cm diameter)

        obstacle = sim.createPrimitiveShape(sim.primitiveshape_cylinder,
                                           [obstacle_radius * 2, obstacle_radius * 2, obstacle_height])
        sim.setObjectPosition(obstacle, -1, [x, y, obstacle_height / 2])
        sim.setObjectColor(obstacle, 0, sim.colorcomponent_ambient_diffuse, [0.8, 0.2, 0.2])
        sim.setObjectAlias(obstacle, f'Obstacle_{x}_{y}')
        return obstacle

    def _create_wall(self, sim, x, y, width, depth, height):
        """Create a single wall segment
        
        Args:
            x, y: position
            width: dimension along X axis
            depth: dimension along Y axis  
            height: dimension along Z axis
        """
        wall = sim.createPrimitiveShape(sim.primitiveshape_cuboid, 
                                       [width, depth, height])
        sim.setObjectPosition(wall, -1, [x, y, height/2])
        sim.setObjectColor(wall, 0, sim.colorcomponent_ambient_diffuse, [0.2, 0.2, 0.8])
        return wall
    
    def _create_guide_line(self, sim, x, y, width, depth, rotation):
        """Create a guide line on the floor showing the path
        
        Args:
            x, y: position
            width: dimension along X axis
            depth: dimension along Y axis
            rotation: rotation around Z axis (not used currently)
        """
        line = sim.createPrimitiveShape(sim.primitiveshape_cuboid, 
                                       [width, depth, 0.001])
        sim.setObjectPosition(line, -1, [x, y, 0.002])
        sim.setObjectColor(line, 0, sim.colorcomponent_ambient_diffuse, [0, 0, 0])
        # Make line non-respondable (non-collidable)
        sim.setObjectInt32Param(line, sim.shapeintparam_respondable, 0)
        return line


def main():
    """Main function to connect to CoppeliaSim and generate maze"""
    print("Connecting to CoppeliaSim...")
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    # Stop simulation if running
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            client.step()
    
    print("Generating maze...")
    # Create maze
    # Use seed parameter for reproducible mazes (same seed = same maze)
    # Example: maze = MazeGenerator(width=8, height=8, cell_size=2.0, seed=42)
    # Leave seed as None (default) for random mazes each time
    maze = MazeGenerator(width=8, height=8, cell_size=2.0, seed=None)
    maze.generate_prim()

    print("Placing obstacles...")
    # Place obstacles in cells NOT on the shortest path
    # Adjust num_obstacles as needed (default is 5)
    maze.place_obstacles(num_obstacles=5)

    print("Creating maze in CoppeliaSim...")
    maze.create_in_coppeliasim(sim)

    print("Maze generation complete!")
    print(f"Maze size: {maze.width}x{maze.height} cells")
    print(f"Cell size: {maze.cell_size}m")
    print(f"Total area: {maze.width * maze.cell_size}m x {maze.height * maze.cell_size}m")
    print(f"Number of obstacles: {len(maze.obstacles)}")
    print("\nYou can now start the simulation")


if __name__ == "__main__":
    main()