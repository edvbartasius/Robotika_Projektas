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
    
    def __init__(self, sim, robot_handle, map_grid, cell_size=2.0):
        self.sim = sim
        self.robot_handle = robot_handle
        self.map_grid = map_grid
        self.cell_size = cell_size
        self.grid_size = len(map_grid)
        self.current_pos = (0, 0)  # (y, x)
        
    def move(self, direction: Direction) -> bool:
        """
        Move robot one cell in the given direction.
        Returns True if successful, False if wall blocks movement.
        Raises ValueError if move would go out of bounds.
        """
        y, x = self.current_pos
        
        # Check wall in this direction
        if self.map_grid[y][x][direction.value]:
            raise ValueError(f"Wall blocks movement {direction.name} from ({y},{x})")
        
        # Calculate new position
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
    navigator = RobotNavigator(sim, robot_handle, map_grid)
    
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
            navigator.set_position(y, x)
            
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
