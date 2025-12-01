local machine = require("statemachine")

function sysCall_init()
    --[=====[
 ######     #    ######  #######  #####
 #     #   # #   #     #    #    #     #
 #     #  #   #  #     #    #    #
 ######  #     # ######     #     #####
 #       ####### #   #      #          #
 #       #     # #    #     #    #     #
 #       #     # #     #    #     #####

--]=====]
    bubbleRobBase = sim.getObject('.')
    leftMotor = sim.getObject("./leftMotor")
    rightMotor = sim.getObject("./rightMotor")
    
    floorSensorHandles = { -1, -1, -1 }
    floorSensorHandles[1] = sim.getObject("./leftSensor")
    floorSensorHandles[2] = sim.getObject("./middleSensor")
    floorSensorHandles[3] = sim.getObject("./rightSensor")
    
    robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, { 1, 1, 0 }, nil, nil, { 1, 1, 0 })

    -- Speed parameters
    minMaxSpeed = { 50 * math.pi / 180, 300 * math.pi / 180 }
    speed = (minMaxSpeed[1] + minMaxSpeed[2]) * 0.5
    
    -- UI
    xml = '<ui title="Speed" closeable="false" resizeable="false" activate="false">' .. [[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui = simUI.create(xml)
    simUI.setSliderValue(ui, 1, 100 * (speed - minMaxSpeed[1]) / (minMaxSpeed[2] - minMaxSpeed[1]))
end

--[=====[
  #####  ######     #    ######  #     #
 #     # #     #   # #   #     # #     #
 #       #     #  #   #  #     # #     #
 #  #### ######  #     # ######  #######
 #     # #   #   ####### #       #     #
 #     # #    #  #     # #       #     #
  #####  #     # #     # #       #     #

--]=====]
function sysCall_sensing()
    local p = sim.getObjectPosition(bubbleRobBase, -1)
    sim.addDrawingObjectItem(robotTrace, p)
end

--[=====[
 #     #    #    ### #     #
 ##   ##   # #    #  ##    #
 # # # #  #   #   #  # #   #
 #  #  # #     #  #  #  #  #
 #     # #######  #  #   # #
 #     # #     #  #  #    ##
 #     # #     # ### #     #

--]=====]
function sysCall_actuation()
    -- Read sensors
    local left, middle, right = false, false, false
    
    local result, data = sim.readVisionSensor(floorSensorHandles[1])
    if result >= 0 then left = data[11] < 0.5 end
    
    result, data = sim.readVisionSensor(floorSensorHandles[2])
    if result >= 0 then middle = data[11] < 0.5 end
    
    result, data = sim.readVisionSensor(floorSensorHandles[3])
    if result >= 0 then right = data[11] < 0.5 end
    
    -- Simple line following with RIGHT priority
    local leftV = speed
    local rightV = speed
    
    if right then
        -- Right sensor sees line - turn RIGHT (priority)
        leftV = speed
        rightV = speed * 0.2
    elseif middle and not left then
        -- Only middle - go straight
        leftV = speed
        rightV = speed
    elseif middle and left then
        -- Middle and left - slight right correction
        leftV = speed
        rightV = speed * 0.7
    elseif left and not middle then
        -- Only left - turn LEFT to find middle
        leftV = speed * 0.2
        rightV = speed
    else
        -- No line - turn right to search
        leftV = speed * 0.5
        rightV = -speed * 0.3
    end
    
    sim.setJointTargetVelocity(leftMotor, leftV)
    sim.setJointTargetVelocity(rightMotor, rightV)
end

function sysCall_cleanup()
    simUI.destroy(ui)
end

--[=====[
 #     # ####### ### #
 #     #    #     #  #
 #     #    #     #  #
 #     #    #     #  #
 #     #    #     #  #
 #     #    #     #  #
  #####     #    ### #######

--]=====]
function speedChange_callback(ui, id, newVal)
    speed = minMaxSpeed[1] + (minMaxSpeed[2] - minMaxSpeed[1]) * newVal / 100
end
