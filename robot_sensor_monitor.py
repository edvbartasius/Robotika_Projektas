from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys
import math

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
        
        # Find all sensors
        print("\nFinding sensors...")
        sensors = find_all_sensors(sim)
        
        print(f"Found {len(sensors)} sensors:")
        for sensor_name, sensor_handle in sensors.items():
            print(f"  - {sensor_name} (Handle: {sensor_handle})")
        
        # Continuous monitoring loop
        print("\n=== Starting Sensor Monitoring (Press Ctrl+C to stop) ===\n")
        
        frame = 0
        try:
            while True:
                sim.step()
                
                # Get robot position and orientation
                pos = sim.getObjectPosition(robot_handle, -1)
                orient = sim.getObjectOrientation(robot_handle, -1)
                
                # Convert orientation to direction (+X, -X, +Y, -Y)
                direction = get_direction_from_orientation(orient)
                
                # Print robot coordinates
                print(f"Frame {frame:04d} | Position: X={pos[0]:7.4f} Y={pos[1]:7.4f} Z={pos[2]:7.4f} | "
                      f"Direction: {direction}", end="")
                
                # Read and display sensor values
                sensor_data = read_all_sensors(sim, sensors)
                
                if sensor_data:
                    sensor_str = " | Sensors: "
                    for sensor_name, distance in sensor_data.items():
                        if distance is not None:
                            sensor_str += f"{sensor_name}={distance:.4f}m "
                        else:
                            sensor_str += f"{sensor_name}=no-detect "
                    print(sensor_str, end="")
                
                print()  # Newline
                
                frame += 1
                time.sleep(0.05)  # ~20 Hz update rate
                
        except KeyboardInterrupt:
            print("\n\nMonitoring stopped by user.")
    
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


def find_all_sensors(sim):
    """Find all proximity and vision sensors in the scene"""
    sensors = {}
    try:
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle)
            obj_type = sim.getObjectType(obj_handle)
            
            # Type 4 is vision sensor, Type 5 is proximity sensor
            # Only keep front sensor
            if (obj_type == 4 or obj_type == 5) and 'front' in obj_name.lower():
                sensors[obj_name] = obj_handle
            
            index += 1
        
        return sensors
    except Exception as e:
        print(f"Error finding sensors: {e}")
        return {}


def read_all_sensors(sim, sensors):
    """Read values from all sensors"""
    sensor_data = {}
    
    for sensor_name, sensor_handle in sensors.items():
        try:
            # Try reading as proximity sensor first
            result = sim.checkProximitySensor(sensor_handle, sim.handle_all)
            
            if result and len(result) >= 2:
                collision_state = result[0]
                distance = result[1]
                if collision_state:
                    sensor_data[sensor_name] = distance
                else:
                    sensor_data[sensor_name] = None
            else:
                sensor_data[sensor_name] = None
                
        except Exception as e:
            # Sensor reading failed
            sensor_data[sensor_name] = None
    
    return sensor_data


def get_direction_from_orientation(orient):
    """Convert Euler angles (roll, pitch, yaw) to direction (+X, -X, +Y, -Y)"""
    # Yaw angle determines the direction
    yaw = orient[2]
    
    # Normalize yaw to 0-2π
    yaw = yaw % (2 * math.pi)
    
    # Determine direction based on yaw angle
    # 0 rad = +X, π/2 rad = +Y, π rad = -X, 3π/2 rad = -Y
    if yaw < math.pi / 4 or yaw >= 7 * math.pi / 4:
        return "+X"
    elif yaw < 3 * math.pi / 4:
        return "+Y"
    elif yaw < 5 * math.pi / 4:
        return "-X"
    else:
        return "-Y"


if __name__ == "__main__":
    main()
