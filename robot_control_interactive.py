from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys
import math
import termios
import tty

def main():
    # Connect to CoppeliaSim
    print("Connecting to CoppeliaSim...")
    try:
        client = RemoteAPIClient()
        client.setStepping(True)
    except Exception as e:
        print(f"Error connecting to CoppeliaSim: {e}")
        print("Make sure CoppeliaSim is running with ZMQ Remote API enabled.")
        return

    sim = client.getObject('sim')
    
    try:
        # Find the robot
        print("Finding robot...")
        robot_handle = find_robot(sim)
        
        if not robot_handle:
            print("No robot found in the scene!")
            return
        
        print(f"Robot found: Handle {robot_handle}")
        
        # Find front sensor
        print("Finding front sensor...")
        front_sensor = find_front_sensor(sim)
        if front_sensor:
            print(f"Front sensor found: Handle {front_sensor}")
        else:
            print("No front sensor found (will continue without it)")
        
        # Get initial position
        pos = sim.getObjectPosition(robot_handle, -1)
        print(f"\nInitial position: X={pos[0]:.4f} Y={pos[1]:.4f} Z={pos[2]:.4f}")
        
        print("\n=== Robot Control ===")
        print("Arrow Keys: Move along X/Y axis")
        print("Up/Down arrows: Move along Y axis")
        print("Left/Right arrows: Move along X axis")
        print("R: Rotate 90° left (counter-clockwise)")
        print("Shift+R: Rotate 90° right (clockwise)")
        print("Press Ctrl+C to exit\n")
        
        # Enable raw input
        old_settings = enable_raw_input()
        
        # Track robot orientation (in radians, 0 = facing +X)
        current_yaw = 0
        
        try:
            while True:
                # Read keyboard input
                char = sys.stdin.read(1)
                
                if char == '\x1b':  # ESC sequence
                    next1 = sys.stdin.read(1)
                    if next1 == '[':
                        direction = sys.stdin.read(1)
                        
                        # Get current position
                        pos = sim.getObjectPosition(robot_handle, -1)
                        
                        # Move robot by 1 unit based on arrow key
                        if direction == 'A':  # Up arrow
                            new_pos = [pos[0], pos[1] + 1.0, pos[2]]
                            print(f"Moving +Y: ", end="")
                        elif direction == 'B':  # Down arrow
                            new_pos = [pos[0], pos[1] - 1.0, pos[2]]
                            print(f"Moving -Y: ", end="")
                        elif direction == 'C':  # Right arrow
                            new_pos = [pos[0] + 1.0, pos[1], pos[2]]
                            print(f"Moving +X: ", end="")
                        elif direction == 'D':  # Left arrow
                            new_pos = [pos[0] - 1.0, pos[1], pos[2]]
                            print(f"Moving -X: ", end="")
                        else:
                            continue
                        
                        # Set new position
                        sim.setObjectPosition(robot_handle, -1, new_pos)
                        sim.step()
                        time.sleep(0.1)
                        
                        # Get updated position
                        updated_pos = sim.getObjectPosition(robot_handle, -1)
                        print(f"Position: X={updated_pos[0]:7.4f} Y={updated_pos[1]:7.4f} Z={updated_pos[2]:7.4f}", end="")
                        
                        # Read front sensor
                        if front_sensor:
                            distance = read_proximity_sensor(sim, front_sensor)
                            if distance is not None:
                                print(f" | Front Sensor: {distance:.4f}m")
                            else:
                                print(f" | Front Sensor: no detection")
                        else:
                            print()
                
                elif char.lower() == 'r':  # Rotation keys
                    is_shift = char.isupper()  # Check if uppercase (Shift+R)
                    
                    if is_shift:
                        # Shift+R: rotate 90° right (clockwise)
                        current_yaw -= math.pi / 2
                        rotation_text = "Rotating 90° right (clockwise)"
                    else:
                        # R: rotate 90° left (counter-clockwise)
                        current_yaw += math.pi / 2
                        rotation_text = "Rotating 90° left (counter-clockwise)"
                    
                    # Normalize yaw to [-π, π]
                    current_yaw = math.atan2(math.sin(current_yaw), math.cos(current_yaw))
                    
                    # Set robot orientation
                    sim.setObjectOrientation(robot_handle, -1, [0, 0, current_yaw])
                    sim.step()
                    time.sleep(0.1)
                    
                    # Get position for display
                    pos = sim.getObjectPosition(robot_handle, -1)
                    print(f"{rotation_text}: ", end="")
                    print(f"Position: X={pos[0]:7.4f} Y={pos[1]:7.4f} Z={pos[2]:7.4f}", end="")
                    print(f" | Yaw: {math.degrees(current_yaw):6.1f}°", end="")
                    
                    # Read front sensor
                    if front_sensor:
                        distance = read_proximity_sensor(sim, front_sensor)
                        if distance is not None:
                            print(f" | Front Sensor: {distance:.4f}m")
                        else:
                            print(f" | Front Sensor: no detection")
                    else:
                        print()
                
                elif char == '\x03':  # Ctrl+C
                    print("\n\nControl stopped by user.")
                    break
        
        finally:
            # Restore terminal settings
            disable_raw_input(old_settings)
    
    finally:
        client.setStepping(False)
        print("Disconnected from CoppeliaSim.")


def find_robot(sim):
    """Find the robot in the scene"""
    try:
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle).lower()
            if 'robot' in obj_name or 'youbot' in obj_name:
                return obj_handle
            
            index += 1
        
        return None
    except Exception as e:
        print(f"Error finding robot: {e}")
        return None


def find_front_sensor(sim):
    """Find the front sensor in the scene"""
    try:
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle).lower()
            obj_type = sim.getObjectType(obj_handle)
            
            # Type 4 is vision sensor, Type 5 is proximity sensor
            if (obj_type == 4 or obj_type == 5) and 'front' in obj_name:
                return obj_handle
            
            index += 1
        
        return None
    except Exception as e:
        print(f"Error finding front sensor: {e}")
        return None


def read_proximity_sensor(sim, sensor_handle):
    """Read distance from proximity sensor"""
    try:
        result = sim.checkProximitySensor(sensor_handle, sim.handle_all)
        
        if result and len(result) >= 2:
            collision_state = result[0]
            distance = result[1]
            if collision_state:
                return distance
        
        return None
    except Exception as e:
        return None


def enable_raw_input():
    """Enable raw input mode for arrow keys"""
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        return old_settings
    except:
        return None


def disable_raw_input(old_settings):
    """Disable raw input mode"""
    if old_settings:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except:
            pass


if __name__ == "__main__":
    main()
