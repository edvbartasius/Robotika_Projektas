function sysCall_init()
    sim=require('sim')
    bubbleRobBase=sim.getObject('..') 
    leftMotor=sim.getObject("../LeftMotor")
    rightMotor=sim.getObject("../RightMotor")
    noseSensor=sim.getObject("../SensingNose")
    distancePrinted = false
    
    -- Set robot position to (1,1,0.1)
    sim.setObjectPosition(bubbleRobBase, {1, 1, 0.1})
    
    -- Stop motors
    sim.setJointTargetVelocity(leftMotor, 0)
    sim.setJointTargetVelocity(rightMotor, 0)
    
    print("Robot positioned at (1,1,0.1)")
end

function sysCall_actuation()
    -- Keep motors stopped
    sim.setJointTargetVelocity(leftMotor, 0)
    sim.setJointTargetVelocity(rightMotor, 0)
    
    if not distancePrinted then
        local result, distance = sim.readProximitySensor(noseSensor)
        if result > 0 then
            print("Distance to nearest wall: " .. string.format("%.3f", distance) .. " meters")
        else
            print("No obstacle detected within sensor range")
        end
        distancePrinted = true
    end
end

