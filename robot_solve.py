from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
from rich.table import Table
from rich.console import Console

def read_proximity_sensor(sim, sensor_handle, num_samples=5):
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
            time.sleep(0.02)
        
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

    # Assume robot starts at (1,1) as cell center, cell size 2m x 2m
    cell_size = 2.0
    start_pos = (0, 0)
    current_pos = list(start_pos)
    current_yaw = 0
    cells_scanned = 0
    total_cells = grid_size * grid_size

    print("\nScanning maze...")
    try:
        for y in range(grid_size):
            for x in range(grid_size):
                current_pos = [y, x]
                # Move robot to cell (x, y)
                # Center of cell: (x+1, y+1) * cell_size
                cell_x = x * cell_size + 1
                cell_y = y * cell_size + 1
                sim.setObjectPosition(robot_handle, -1, [cell_x, cell_y, 0.138])
                sim.setObjectOrientation(robot_handle, -1, [0, 0, 0])
                sim.step()
                time.sleep(0.1)

                # Scan all 4 directions for walls
                for dir_name, wall_idx, yaw in directions:
                    sim.setObjectOrientation(robot_handle, -1, [0, 0, yaw])
                    sim.step()
                    time.sleep(0.05)
                    dist = read_proximity_sensor(sim, front_sensor)
                    wall = is_wall_detected(dist)
                    map_grid[y][x][wall_idx] = wall
                cells_scanned += 1
                display_map(map_grid)
                print(f"Scanned cell ({y},{x})")
        print("\n✓ Maze scan complete!")
        display_map(map_grid)
    except KeyboardInterrupt:
        print("\n\n✗ Scan interrupted by user")
        print(f"Partial results: {cells_scanned}/{total_cells} cells scanned")
        display_map(map_grid)
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
    finally:
        client.setStepping(False)
        print("\nDisconnected from CoppeliaSim.")

if __name__ == "__main__":
    main()
