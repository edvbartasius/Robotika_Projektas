from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

class MazeRobot:
    CELL_SIZE = 2.0  # Each cell is 2m x 2m
    MAZE_SIZE = 8    # 8x8 grid
    
    def __init__(self):
        self.client = None
        self.sim = None
        self.robot_handle = None
        self.front_sensor = None
        self.bottom_sensor = None
        
        # Maze map: False = passable, True = obstacle/wall
        self.maze_map = [[False for _ in range(self.MAZE_SIZE)] for _ in range(self.MAZE_SIZE)]
        
        # Robot state
        self.current_cell = [1, 1]  # Start at center (0-indexed from corner)
        self.current_direction = 0  # 0=+X, 1=+Y, 2=-X, 3=-Y
    
    def connect(self):
        """Connect to CoppeliaSim"""
        print("Connecting to CoppeliaSim...")
        try:
            self.client = RemoteAPIClient()
            self.client.setStepping(True)
            self.sim = self.client.getObject('sim')
            print("Connected successfully!")
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
        
        return True
    
    def find_robot(self):
        """Find the robot in the scene"""
        try:
            index = 0
            while True:
                obj_handle = self.sim.getObjects(index, self.sim.handle_all)
                if obj_handle == -1:
                    break
                
                obj_name = self.sim.getObjectName(obj_handle).lower()
                if 'robot' in obj_name or 'youbot' in obj_name:
                    self.robot_handle = obj_handle
                    print(f"Robot found: {self.sim.getObjectName(obj_handle)}")
                    return True
                
                index += 1
            
            print("No robot found!")
            return False
        except Exception as e:
            print(f"Error finding robot: {e}")
            return False
    
    def find_sensors(self):
        """Find front and bottom sensors"""
        try:
            index = 0
            while True:
                obj_handle = self.sim.getObjects(index, self.sim.handle_all)
                if obj_handle == -1:
                    break
                
                obj_name = self.sim.getObjectName(obj_handle).lower()
                obj_type = self.sim.getObjectType(obj_handle)
                
                # Type 4 is vision sensor, Type 5 is proximity sensor
                if obj_type == 4 or obj_type == 5:
                    if 'front' in obj_name:
                        self.front_sensor = obj_handle
                        print(f"Front sensor found: {self.sim.getObjectName(obj_handle)}")
                    elif 'bottom' in obj_name:
                        self.bottom_sensor = obj_handle
                        print(f"Bottom sensor found: {self.sim.getObjectName(obj_handle)}")
                
                index += 1
            
            return True
        except Exception as e:
            print(f"Error finding sensors: {e}")
            return False
    
    def read_sensor(self, sensor_handle):
        """Read proximity sensor value"""
        try:
            result = self.sim.checkProximitySensor(sensor_handle, self.sim.handle_all)
            
            if result and len(result) >= 2:
                collision_state = result[0]
                distance = result[1]
                if collision_state:
                    return distance
            
            return None
        except Exception as e:
            return None
    
    def get_robot_position(self):
        """Get robot's current position in simulation"""
        try:
            pos = self.sim.getObjectPosition(self.robot_handle, -1)
            return pos
        except Exception as e:
            print(f"Error getting position: {e}")
            return None
    
    def set_robot_position(self, x, y, z=0.05):
        """Set robot's position in simulation"""
        try:
            self.sim.setObjectPosition(self.robot_handle, -1, [x, y, z])
            self.sim.step()
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"Error setting position: {e}")
            return False
    
    def set_robot_orientation(self, yaw):
        """Set robot's orientation (rotation around Z axis)"""
        try:
            # Get current position to keep it
            pos = self.sim.getObjectPosition(self.robot_handle, -1)
            # Set orientation: roll=0, pitch=0, yaw=specified
            self.sim.setObjectOrientation(self.robot_handle, -1, [0, 0, yaw])
            self.sim.step()
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"Error setting orientation: {e}")
            return False
    
    def get_direction_string(self, direction):
        """Convert direction number to string"""
        directions = ["+X", "+Y", "-X", "-Y"]
        return directions[direction % 4]
    
    def cell_to_world_coordinates(self, cell_x, cell_y):
        """Convert cell coordinates to world coordinates"""
        # Cell (0,0) is at world (1, 1) - center of first cell
        world_x = 1.0 + cell_x * self.CELL_SIZE
        world_y = 1.0 + cell_y * self.CELL_SIZE
        return world_x, world_y
    
    def world_to_cell_coordinates(self, world_x, world_y):
        """Convert world coordinates to cell coordinates"""
        cell_x = int((world_x - 1.0) / self.CELL_SIZE)
        cell_y = int((world_y - 1.0) / self.CELL_SIZE)
        return cell_x, cell_y
    
    def move_to_cell(self, cell_x, cell_y):
        """Move robot to a specific cell"""
        world_x, world_y = self.cell_to_world_coordinates(cell_x, cell_y)
        
        print(f"Moving to cell ({cell_x}, {cell_y}) -> world ({world_x:.2f}, {world_y:.2f})", end="")
        
        if self.set_robot_position(world_x, world_y):
            # Read sensors
            front_dist = self.read_sensor(self.front_sensor) if self.front_sensor else None
            
            pos = self.get_robot_position()
            if pos:
                print(f" | Position: ({pos[0]:.2f}, {pos[1]:.2f})", end="")
            
            if front_dist is not None:
                print(f" | Front: {front_dist:.4f}m", end="")
            
            print()
            self.current_cell = [cell_x, cell_y]
            return True
        
        return False
    
    def explore_maze(self):
        """Systematically explore the maze and scan walls"""
        print("\n=== Starting Maze Exploration ===")
        print(f"Maze size: {self.MAZE_SIZE}x{self.MAZE_SIZE} cells of {self.CELL_SIZE}m x {self.CELL_SIZE}m")
        print(f"Starting position: Cell (1, 1)\n")
        
        # Move to starting position
        self.move_to_cell(1, 1)
        time.sleep(0.2)
        
        # Explore the maze in a pattern
        # Row by row pattern with wall scanning
        for y in range(self.MAZE_SIZE):
            if y % 2 == 0:
                # Left to right
                for x in range(self.MAZE_SIZE):
                    self.move_to_cell(x, y)
                    self.scan_walls_at_cell(x, y)
                    time.sleep(0.1)
            else:
                # Right to left
                for x in range(self.MAZE_SIZE - 1, -1, -1):
                    self.move_to_cell(x, y)
                    self.scan_walls_at_cell(x, y)
                    time.sleep(0.1)
        
        print("\n=== Maze Exploration Complete ===")
        
        # Return to start
        print("Returning to start...")
        self.move_to_cell(1, 1)
    
    def scan_walls_at_cell(self, cell_x, cell_y):
        """Scan walls in all four directions from current cell"""
        if not self.front_sensor:
            return
        
        print(f"  Scanning walls at cell ({cell_x}, {cell_y}):", end=" ")
        
        directions = [0, 1, 2, 3]  # +X, +Y, -X, -Y
        direction_names = ["+X", "+Y", "-X", "-Y"]
        yaw_angles = [0, math.pi/2, math.pi, 3*math.pi/2]
        
        wall_detected = {}
        
        for direction, yaw in zip(directions, yaw_angles):
            # Rotate robot to face this direction
            self.set_robot_orientation(yaw)
            time.sleep(0.05)
            
            # Read sensor
            distance = self.read_sensor(self.front_sensor)
            
            if distance is not None and distance < 2.5:  # Wall detected (sensor range)
                wall_detected[direction_names[direction]] = distance
        
        if wall_detected:
            wall_str = ", ".join([f"{dir}({dist:.2f}m)" for dir, dist in wall_detected.items()])
            print(f"Walls: {wall_str}")
        else:
            print("No walls detected")
        
        # Rotate back to +X direction
        self.set_robot_orientation(0)
        time.sleep(0.05)
    
    def print_maze_status(self):
        """Print the current maze map"""
        print("\n=== Maze Map ===")
        print("(. = passable, X = obstacle/wall)")
        print("  ", end="")
        for x in range(self.MAZE_SIZE):
            print(f" {x}", end="")
        print()
        
        for y in range(self.MAZE_SIZE - 1, -1, -1):  # Print top to bottom
            print(f"{y} ", end="")
            for x in range(self.MAZE_SIZE):
                if x == self.current_cell[0] and y == self.current_cell[1]:
                    print(" R", end="")  # R for robot
                elif self.maze_map[y][x]:
                    print(" X", end="")  # Obstacle
                else:
                    print(" .", end="")  # Passable
            print()
    
    def run(self):
        """Main execution loop"""
        try:
            # Connect and initialize
            if not self.connect():
                return
            
            if not self.find_robot():
                return
            
            self.find_sensors()
            
            # Run maze exploration
            self.explore_maze()
            
            # Print final status
            self.print_maze_status()
            
            print("\nMaze navigation complete!")
        
        except KeyboardInterrupt:
            print("\n\nInterrupted by user.")
        
        except Exception as e:
            print(f"Error during execution: {e}")
        
        finally:
            self.disconnect()
    
    def disconnect(self):
        """Disconnect from CoppeliaSim"""
        try:
            if self.client:
                self.client.setStepping(False)
            print("Disconnected from CoppeliaSim.")
        except Exception as e:
            print(f"Error disconnecting: {e}")


if __name__ == "__main__":
    robot = MazeRobot()
    robot.run()
