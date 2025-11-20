from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

def read_proximity_sensor(sim, sensor_handle):
    try:
        result = sim.checkProximitySensor(sensor_handle, sim.handle_all)
        if result and len(result) >= 2:
            collision_state = result[0]
            distance = result[1]
            if collision_state:
                return distance
        return None
    except Exception:
        return None

def is_wall_detected(distance):
    return distance is not None and distance <= 1.1

def display_map(map_grid):
    # Print top border
    print("+" + "---+" * 8)
    for y in range(8):
        # Print cell row with vertical walls
        row_str = "|"
        for x in range(8):
            row_str += "   "  # Empty cell
            if x < 7:
                if map_grid[y][x][1]:  # Right wall
                    row_str += "|"
                else:
                    row_str += " "
            else:
                row_str += "|"  # Rightmost wall
        print(row_str)
        # Print horizontal walls below, except after last row
        if y < 7:
            horiz_str = "+"
            for x in range(8):
                if map_grid[y][x][2]:  # Down wall
                    horiz_str += "---"
                else:
                    horiz_str += "   "
                horiz_str += "+"
            print(horiz_str)
    # Print bottom border
    print("+" + "---+" * 8)
    print()

def main():
    print("Connecting to CoppeliaSim...")
    try:
        client = RemoteAPIClient()
        client.setStepping(True)
    except Exception as e:
        print(f"Error connecting to CoppeliaSim: {e}")
        return
    sim = client.getObject('sim')
    # Find robot
    try:
        robot_handle = sim.getObject('/BubbleRobot')
    except Exception:
        robot_handle = -1

    if robot_handle == -1:
        print("No robot found!")
        return

    # Find motors and sensors
    try:
        front_sensor = sim.getObject('./SensingNose')
        left_motor = sim.getObject('./LeftMotor')
        right_motor = sim.getObject('./RightMotor')
    except Exception as e:
        print(f"Error getting handles: {e}")
        return

    if front_sensor == -1:
        print("No front sensor found!")
        return
    if left_motor == -1 or right_motor == -1:
        print("Motors not found!")
        return
    # Map: 8x8 grid, each cell [up, right, down, left] (bools)
    map_grid = [[[False, False, False, False] for _ in range(8)] for _ in range(8)]
    # Directions: (dx, dy, yaw)
    directions = [
        (0, 1, 0),      # Up
        (1, 0, math.pi/2),  # Right
        (0, -1, math.pi),   # Down
        (-1, 0, -math.pi/2) # Left
    ]
    # Scan each cell
    for y in range(8):
        for x in range(8):
            # Move robot to cell center
            cell_x = x * 2 + 1
            cell_y = y * 2 + 1
            sim.setObjectPosition(robot_handle, -1, [cell_x, cell_y, 0.1])
            sim.setObjectOrientation(robot_handle, -1, [0, 0, 0])
            sim.step()
            time.sleep(0.2)
            # Scan 4 directions
            for dir_idx, (dx, dy, yaw) in enumerate(directions):
                sim.setObjectOrientation(robot_handle, -1, [0, 0, yaw])
                sim.step()
                time.sleep(0.1)
                distance = read_proximity_sensor(sim, front_sensor)
                wall = is_wall_detected(distance)
                map_grid[y][x][dir_idx] = wall
            display_map(map_grid)
            time.sleep(0.1)
    client.setStepping(False)
    print("Scan complete.")

if __name__ == "__main__":
    main()
