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
                print(f"  - {obj_name} (Handle: {obj_handle})")
            
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
    """Demonstrate moving the robot"""
    try:
        print(f"\n=== Demonstrating Robot Movement ===")
        
        # Get current position
        pos = sim.getObjectPosition(robot_handle, -1)
        print(f"Current position: {pos}")
        
        # Get current orientation (Euler angles)
        orient = sim.getObjectOrientation(robot_handle, -1)
        print(f"Current orientation: {orient}")
        
        # Move the robot forward (along X axis)
        print("\nMoving robot in X direction...")
        new_x = pos[0] + 0.5
        sim.setObjectPosition(robot_handle, -1, [new_x, pos[1], pos[2]])
        sim.step()
        time.sleep(0.5)
        
        # Get new position
        new_pos = sim.getObjectPosition(robot_handle, -1)
        print(f"New position: {new_pos}")
        
        # Move in Y direction
        print("\nMoving robot in Y direction...")
        new_y = pos[1] + 0.5
        sim.setObjectPosition(robot_handle, -1, [new_x, new_y, pos[2]])
        sim.step()
        time.sleep(0.5)
        
        # Get position
        new_pos = sim.getObjectPosition(robot_handle, -1)
        print(f"New position: {new_pos}")
        
        # Rotate the robot
        print("\nRotating robot...")
        import math
        new_orient = [orient[0], orient[1], orient[2] + math.pi / 4]  # 45 degree rotation
        sim.setObjectOrientation(robot_handle, -1, new_orient)
        sim.step()
        time.sleep(0.5)
        
        # Return to original position
        print("\nReturning robot to original position...")
        sim.setObjectPosition(robot_handle, -1, pos)
        sim.setObjectOrientation(robot_handle, -1, orient)
        sim.step()
        time.sleep(0.5)
        
        final_pos = sim.getObjectPosition(robot_handle, -1)
        print(f"Final position (should match original): {final_pos}")
        
    except Exception as e:
        print(f"Error demonstrating robot movement: {e}")


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
