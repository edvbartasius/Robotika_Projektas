from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
from rich.table import Table
from rich.console import Console
from collections import deque
from enum import Enum

class Direction(Enum):
    UP = 0      # +Y
    RIGHT = 1   # +X
    DOWN = 2    # -Y
    LEFT = 3    # -X

class RobotNavigator:
    """Manages robot movement with wall collision detection"""
    
    def __init__(self, sim, robot_handle, map_grid, sensor_handle, cell_size=2.0):
        self.sim = sim
        self.robot_handle = robot_handle
        self.map_grid = map_grid
        self.sensor_handle = sensor_handle
        self.cell_size = cell_size
        self.grid_size = len(map_grid)
        self.current_pos = (0, 0)  # (y, x)
    
    def read_sensor_in_direction(self, direction: Direction) -> bool:
        """Read sensor in specified direction and return True if wall detected"""
        # Save current orientation
        current_orient = self.sim.getObjectOrientation(self.robot_handle, -1)
        
        # Set orientation for the direction
        direction_orientations = {
            Direction.UP: [0, 0, math.pi/2],     # +Y
            Direction.RIGHT: [0, 0, 0],          # +X
            Direction.DOWN: [0, 0, -math.pi/2],  # -Y
            Direction.LEFT: [0, 0, math.pi]      # -X
        }
        
        self.sim.setObjectOrientation(self.robot_handle, -1, direction_orientations[direction])
        self.sim.step()
        time.sleep(0.05)
        
        # Read sensor
        distance = read_proximity_sensor(self.sim, self.sensor_handle)
        wall_detected = is_wall_detected(distance)
        
        # Restore orientation
        self.sim.setObjectOrientation(self.robot_handle, -1, current_orient)
        self.sim.step()
        
        return wall_detected
        
    def move(self, direction: Direction) -> bool:
        """
        Move robot one cell in the given direction.
        Uses sensors to confirm no wall blocks movement.
        Returns True if successful, False if wall blocks movement.
        Raises ValueError if move would go out of bounds.
        """
        y, x = self.current_pos
        
        # Calculate new position first to check bounds
        if direction == Direction.UP:
            new_y, new_x = y + 1, x
        elif direction == Direction.RIGHT:
            new_y, new_x = y, x + 1
        elif direction == Direction.DOWN:
            new_y, new_x = y - 1, x
        elif direction == Direction.LEFT:
            new_y, new_x = y, x - 1
        
        # Check bounds
        if not (0 <= new_x < self.grid_size and 0 <= new_y < self.grid_size):
            raise ValueError(f"Movement would go out of bounds to ({new_y},{new_x})")
        
        # Use sensor to check for wall in movement direction
        if self.read_sensor_in_direction(direction):
            return False  # Wall detected, cannot move
        
        # Move robot
        cell_x = new_x * self.cell_size + 1
        cell_y = new_y * self.cell_size + 1
        self.sim.setObjectPosition(self.robot_handle, -1, [cell_x, cell_y, 0.138])
        self.sim.step()
        time.sleep(0.1)
        
        self.current_pos = (new_y, new_x)
        return True
    
    def get_position(self):
        """Get current cell position (y, x)"""
        return self.current_pos
    
    def get_accessible_directions(self) -> list:
        """Get list of directions with no walls"""
        y, x = self.current_pos
        accessible = []
        for direction in Direction:
            if not self.map_grid[y][x][direction.value]:
                accessible.append(direction)
        return accessible
    
    def set_position(self, y, x):
        """Teleport robot to cell (y, x) for scanning - bypasses wall checks"""
        if not (0 <= x < self.grid_size and 0 <= y < self.grid_size):
            raise ValueError(f"Position ({y},{x}) out of bounds")
        
        cell_x = x * self.cell_size + 1
        cell_y = y * self.cell_size + 1
        self.sim.setObjectPosition(self.robot_handle, -1, [cell_x, cell_y, 0.138])
        self.sim.setObjectOrientation(self.robot_handle, -1, [0, 0, 0])
        self.sim.step()
        time.sleep(0.1)
        self.current_pos = (y, x)

    def navigate_to(self, target_y, target_x):
        """
        Navigate to target cell using BFS pathfinding.
        Returns True if navigation successful, False if path blocked or target unreachable.
        """
        start_y, start_x = self.current_pos
        
        # Check if already at target
        if (start_y, start_x) == (target_y, target_x):
            return True
        
        # Check if target is within bounds
        if not (0 <= target_x < self.grid_size and 0 <= target_y < self.grid_size):
            raise ValueError(f"Target ({target_y},{target_x}) out of bounds")
        
        # BFS to find shortest path
        queue = deque([(start_y, start_x, [])])
        visited = {(start_y, start_x)}
        
        while queue:
            current_y, current_x, path = queue.popleft()
            
            # Check all four directions
            for direction in Direction:
                if direction == Direction.UP:
                    next_y, next_x = current_y + 1, current_x
                elif direction == Direction.RIGHT:
                    next_y, next_x = current_y, current_x + 1
                elif direction == Direction.DOWN:
                    next_y, next_x = current_y - 1, current_x
                elif direction == Direction.LEFT:
                    next_y, next_x = current_y, current_x - 1
                
                # Check bounds
                if not (0 <= next_x < self.grid_size and 0 <= next_y < self.grid_size):
                    continue
                    
                # Skip if already visited
                if (next_y, next_x) in visited:
                    continue
                
                # Temporarily move to current cell to check if path is clear
                original_pos = self.current_pos
                self.set_position(current_y, current_x)
                
                # Check if we can move in this direction using sensors
                can_move = not self.read_sensor_in_direction(direction)
                
                # Restore original position
                self.set_position(original_pos[0], original_pos[1])
                
                if can_move:
                    new_path = path + [direction]
                    
                    # Check if we reached the target
                    if (next_y, next_x) == (target_y, target_x):
                        # Execute the path
                        for move_direction in new_path:
                            if not self.move(move_direction):
                                print(f"Path blocked during execution at {self.current_pos}")
                                return False
                        return True
                    
                    visited.add((next_y, next_x))
                    queue.append((next_y, next_x, new_path))
        
        print(f"No path found from ({start_y},{start_x}) to ({target_y},{target_x})")
        return False

def navigate_to_global(navigator, x, y):
    """Global navigate_to function for backward compatibility"""
    return navigator.navigate_to(y, x)  # Note: swapped x,y to y,x for internal consistency

def read_proximity_sensor(sim, sensor_handle, num_samples=1):
    """Read proximity sensor with multiple samples for reliability"""
    try:
        distances = []
        for _ in range(num_samples):
            result = sim.checkProximitySensor(sensor_handle, sim.handle_all)
            if result and len(result) >= 2:
                collision_state = result[0]
                distance = result[1]
                if collision_state:
                    distances.append(distance)
            sim.step()
        
        # Return median distance if we have readings
        if distances:
            distances.sort()
            return distances[len(distances) // 2]
        return None
    except Exception as e:
        print(f"Sensor error: {e}")
        return None

def is_wall_detected(distance, threshold=1.1):
    """Check if wall is detected within threshold distance"""
    return distance is not None and distance <= threshold

def display_map(map_grid, current_pos=None):
    console = Console()
    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Y/X", justify="center")
    for x in range(len(map_grid[0])):
        table.add_column(str(x), justify="center")

    for y in range(len(map_grid)-1, -1, -1):
        row = [str(y)]
        for x in range(len(map_grid[y])):
            cell = map_grid[y][x]
            # cell: [up, right, down, left]
            options = ""
            if not cell[0]: options += "↑"
            if not cell[1]: options += "→"
            if not cell[2]: options += "↓"
            if not cell[3]: options += "←"
            if current_pos and (y, x) == current_pos:
                options = f"[bold yellow]R[/]" + options
            row.append(options if options else " ")
        table.add_row(*row)
    console.print(table)

def main():
    print("=" * 70)
    print("MAZE SCANNER - Robot Solve")
    print("=" * 70)
    print("\nConnecting to CoppeliaSim...")
    
    try:
        client = RemoteAPIClient()
        client.setStepping(True)
        sim = client.getObject('sim')
        print("✓ Connected to CoppeliaSim")
    except Exception as e:
        print(f"✗ Error connecting to CoppeliaSim: {e}")
        print("  Make sure CoppeliaSim is running with ZMQ Remote API enabled.")
        return
    
    # Find robot and sensors
    try:
        robot_handle = sim.getObject('/BubbleRobot')
        print(f"✓ Robot found (Handle: {robot_handle})")
    except Exception as e:
        print(f"✗ No robot found: {e}")
        return

    try:
        front_sensor = sim.getObject('./SensingNose')
        print(f"✓ Front sensor found (Handle: {front_sensor})")
    except Exception as e:
        print(f"✗ Error getting sensor: {e}")
        return

    # Initialize map grid
    grid_size = 8
    map_grid = [[[False, False, False, False] for _ in range(grid_size)] for _ in range(grid_size)]
    directions = [
        ("Up (+Y)",    0, math.pi/2),     # Index 0: Up wall - face +Y
        ("Right (+X)", 1, 0),             # Index 1: Right wall - face +X  
        ("Down (-Y)",  2, -math.pi/2),    # Index 2: Down wall - face -Y
        ("Left (-X)",  3, math.pi)        # Index 3: Left wall - face -X
    ]

    # Create navigator
    navigator = RobotNavigator(sim, robot_handle, map_grid, front_sensor)
    
    # Assume robot starts at (1,1) as cell center, cell size 2m x 2m
    cell_size = 2.0
    start_cell = (0, 0)  # cell (0,0)
    visited = set([start_cell])
    stack = [start_cell]
    cells_scanned = 0

    print("\nScanning maze...")
    try:
        while stack:
            current_cell = stack.pop()
            y, x = current_cell
            
            # Move robot to cell for scanning
            navigator.navigate_to(y, x)
            
            # Scan all 4 directions for walls
            for dir_name, wall_idx, yaw in directions:
                sim.setObjectOrientation(robot_handle, -1, [0, 0, yaw])
                sim.step()
                dist = read_proximity_sensor(sim, front_sensor)
                wall = is_wall_detected(dist)
                map_grid[y][x][wall_idx] = wall
            
            cells_scanned += 1
            print(f"Scanned cell ({y},{x})")
            display_map(map_grid)
            input()

            # Add accessible neighbors to stack
            neighbor_moves = [
                (Direction.UP, 0),      # wall_idx=0
                (Direction.RIGHT, 1),   # wall_idx=1
                (Direction.DOWN, 2),    # wall_idx=2
                (Direction.LEFT, 3)     # wall_idx=3
            ]
            for direction, wall_idx in neighbor_moves:
                if not map_grid[y][x][wall_idx]:  # no wall in this direction
                    if direction == Direction.UP:
                        ny, nx = y + 1, x
                    elif direction == Direction.RIGHT:
                        ny, nx = y, x + 1
                    elif direction == Direction.DOWN:
                        ny, nx = y - 1, x
                    else:  # LEFT
                        ny, nx = y, x - 1
                    
                    if 0 <= nx < grid_size and 0 <= ny < grid_size and (ny, nx) not in visited:
                        visited.add((ny, nx))
                        stack.append((ny, nx))
                        print(f"  Added cell ({ny},{nx}) to scan stack")
        
        print("\n✓ Maze scan complete!")
        display_map(map_grid)
    except KeyboardInterrupt:
        print("\n\n✗ Scan interrupted by user")
        print(f"Partial results: {cells_scanned} cells scanned")
        display_map(map_grid)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
    finally:
        client.setStepping(False)
        print("\nDisconnected from CoppeliaSim.")

if __name__ == "__main__":
    main()
