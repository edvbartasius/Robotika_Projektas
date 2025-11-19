from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

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
        # List all mesh entities in the scene
        print("\n=== Listing Mesh Entities ===")
        list_mesh_entities(sim)
        
        # Find and demonstrate robot control
        print("\n=== Finding Robot ===")
        robot = find_robot(sim)
        
        if robot:
            print(f"Robot found: {robot}")
            demonstrate_robot_movement(sim, robot)
        else:
            print("No robot found in the scene. Listing all objects...")
            list_all_objects(sim)
    
    finally:
        client.setStepping(False)


def list_mesh_entities(sim):
    """List all mesh entities in the scene"""
    try:
        # Use handle_all to get all objects, then filter for shapes
        print("Listing all shape objects:")
        
        shapes = []
        index = 0
        while True:
            # Get shape objects (type = 0 in simGetObjectType)
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_type = sim.getObjectType(obj_handle)
            if obj_type == 0:  # Shape type
                shapes.append(obj_handle)
                obj_name = sim.getObjectName(obj_handle)
                # print(f"  - {obj_name} (Handle: {obj_handle})")
            
            index += 1
        
        print(f"Found {len(shapes)} shape objects")
        
    except Exception as e:
        print(f"Error listing mesh entities: {e}")


def find_robot(sim):
    """Find the robot in the scene"""
    try:
        # Iterate through all objects to find robot
        print("Searching for robot...")
        
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle).lower()
            if 'robot' in obj_name or 'youbot' in obj_name:
                return obj_handle
            
            index += 1
        
        # If no robot found, list all objects
        print("No robot found. Listing all objects in scene:")
        
        index = 0
        type_names = {0: "Shape", 1: "Joint", 2: "Dummy", 3: "Script", 4: "Vision", 5: "Proximity", 6: "Graph"}
        
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle)
            obj_type = sim.getObjectType(obj_handle)
            type_name = type_names.get(obj_type, f"Type {obj_type}")
            print(f"  - {obj_name} ({type_name})")
            
            index += 1
        
        return None
    except Exception as e:
        print(f"Error finding robot: {e}")
        return None



def demonstrate_robot_movement(sim, robot_handle):
    """Demonstrate moving the robot and detect black line and blue wall"""
    try:
        print(f"\n=== Demonstrating Robot Movement ===")
        
        # Get current position
        pos = sim.getObjectPosition(robot_handle, -1)
        print(f"Current position: {pos}")
        
        # Get current orientation (Euler angles)
        orient = sim.getObjectOrientation(robot_handle, -1)
        print(f"Current orientation: {orient}")
        
        # Check for proximity sensors (for front wall detection and black line detection)
        print("\n=== Checking for Sensors ===")
        front_sensor = find_sensor_by_name(sim, "front")
        bottom_sensor = find_sensor_by_name(sim, "bottom")
        
        # Continuous sensor reading loop
        print("\n=== Sensor Readings ===")
        for i in range(2000):
            sim.step()
            
            # Read front sensor (blue wall distance)
            if front_sensor:
                distance_front = read_proximity_sensor(sim, front_sensor)
                if distance_front is not None:
                    print(f"Frame {i}: Front sensor distance to blue wall: {distance_front:.3f} m")
                else:
                    print(f"Frame {i}: Front sensor - no detection")
            
            # Read bottom sensor (black line detection)
            if bottom_sensor:
                distance_bottom = read_proximity_sensor(sim, bottom_sensor)
                if distance_bottom is not None:
                    print(f"Frame {i}: Bottom sensor distance to black line: {distance_bottom:.3f} m")
                    # Black line detection - if distance is very small
                    if distance_bottom < 0.01:
                        print(f"              *** BLACK LINE DETECTED ***")
                else:
                    print(f"Frame {i}: Bottom sensor - no black line detected")
            
            time.sleep(0.1)
        
    except Exception as e:
        print(f"Error demonstrating robot movement: {e}")


def find_sensor_by_name(sim, sensor_type):
    """Find a sensor by searching for objects containing the sensor type in their name"""
    try:
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle).lower()
            obj_type = sim.getObjectType(obj_handle)
            
            # Type 5 is proximity sensor, type 4 is vision sensor
            if (obj_type == 5 or obj_type == 4) and sensor_type in obj_name:
                print(f"Found {sensor_type} sensor: {sim.getObjectName(obj_handle)} (Handle: {obj_handle})")
                return obj_handle
            
            index += 1
        
        print(f"No {sensor_type} sensor found")
        return None
    except Exception as e:
        print(f"Error finding {sensor_type} sensor: {e}")
        return None


def read_proximity_sensor(sim, sensor_handle):
    """Read distance from proximity sensor"""
    try:
        # Get proximity sensor state
        result = sim.checkProximitySensor(sensor_handle, sim.handle_all)
        
        # Result is [collision_state, distance, collision_point, ...]
        if result and len(result) >= 2:
            collision_state = result[0]
            distance = result[1]
            if collision_state:
                return distance
        
        return None
    except Exception as e:
        # If checkProximitySensor fails, try alternative method
        try:
            # Try reading vision sensor
            data = sim.getVisionSensorImage(sensor_handle)
            return data
        except:
            return None


def list_all_objects(sim):
    """List all objects in the scene"""
    try:
        print("\nAll objects in scene:")
        
        type_names = {0: "Shape", 1: "Joint", 2: "Dummy", 3: "Script", 4: "Vision", 5: "Proximity", 6: "Graph"}
        
        index = 0
        while True:
            obj_handle = sim.getObjects(index, sim.handle_all)
            if obj_handle == -1:
                break
            
            obj_name = sim.getObjectName(obj_handle)
            obj_type = sim.getObjectType(obj_handle)
            type_name = type_names.get(obj_type, f"Type {obj_type}")
            print(f"  - {obj_name} ({type_name})")
            
            index += 1
    except Exception as e:
        print(f"Error listing objects: {e}")


if __name__ == "__main__":
    main()
