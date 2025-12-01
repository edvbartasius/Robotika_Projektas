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
    noseSensor = sim.getObject("./sensingNose")
    frontLeftSensor = sim.getObject("./frontLeftSensor")
    frontRightSensor = sim.getObject("./frontRightSensor")
    
    floorSensorHandles = { -1, -1, -1 }
    floorSensorHandles[1] = sim.getObject("./leftSensor")
    floorSensorHandles[2] = sim.getObject("./middleSensor")
    floorSensorHandles[3] = sim.getObject("./rightSensor")
    
    robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, { 1, 1, 0 }, nil, nil, { 1, 1, 0 })

    -- Speed parameters
    minMaxSpeed = { 50 * math.pi / 180, 300 * math.pi / 180 }
    speed = (minMaxSpeed[1] + minMaxSpeed[2]) * 0.5
    
    -- Obstacle avoidance state
    avoidingObstacle = false
    avoidancePhase = 0  -- 0: not avoiding, 1: turn left, 2: go forward, 3: turn right, 4: find line
    avoidanceStartTime = 0
    avoidancePhaseDuration = { 0.5, 0.8, 0.5, 1.0 }  -- Duration for each phase
    
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
    local currentTime = sim.getSimulationTime()
    
    -- Read proximity sensors for obstacles
    local noseDetected = sim.readProximitySensor(noseSensor) > 0
    local frontLeftDetected = sim.readProximitySensor(frontLeftSensor) > 0
    local frontRightDetected = sim.readProximitySensor(frontRightSensor) > 0
    local obstacleDetected = noseDetected or frontLeftDetected or frontRightDetected
    
    -- Read floor sensors
    local left, middle, right = false, false, false
    local result, data = sim.readVisionSensor(floorSensorHandles[1])
    if result >= 0 then left = data[11] < 0.5 end
    
    result, data = sim.readVisionSensor(floorSensorHandles[2])
    if result >= 0 then middle = data[11] < 0.5 end
    
    result, data = sim.readVisionSensor(floorSensorHandles[3])
    if result >= 0 then right = data[11] < 0.5 end
    
    local leftV, rightV
    
    -- Check for obstacle - start avoidance
    if obstacleDetected and not avoidingObstacle then
        avoidingObstacle = true
        avoidancePhase = 1
        avoidanceStartTime = currentTime
        print(">>> Obstacle detected! Starting avoidance...")
    end
    
    if avoidingObstacle then
        -- Smart obstacle avoidance using front sensors
        local phaseTime = currentTime - avoidanceStartTime
        
        if avoidancePhase == 1 then
            -- Phase 1: Turn left until front is clear
            leftV = -speed * 0.5
            rightV = speed * 0.5
            if not noseDetected and not frontRightDetected and phaseTime > 0.3 then
                avoidancePhase = 2
                avoidanceStartTime = currentTime
                print(">>> Avoidance phase 2: Wall follow")
            end
        elseif avoidancePhase == 2 then
            -- Phase 2: Wall follow - keep obstacle on right side
            if frontRightDetected or noseDetected then
                -- Obstacle still on right or ahead - turn left
                leftV = speed * 0.3
                rightV = speed * 0.6
            elseif frontLeftDetected then
                -- Something on left - turn right
                leftV = speed * 0.6
                rightV = speed * 0.3
            else
                -- Clear ahead - go forward and slight right to stay near wall
                leftV = speed * 0.7
                rightV = speed * 0.5
            end
            
            -- Check if we found the line (and obstacle is clear)
            if (middle or left or right) and not obstacleDetected then
                avoidingObstacle = false
                avoidancePhase = 0
                print(">>> Line found! Resuming normal operation.")
            elseif phaseTime > 3.0 then
                -- Timeout - switch to search mode
                avoidancePhase = 3
                avoidanceStartTime = currentTime
                print(">>> Avoidance timeout - searching for line")
            end
        elseif avoidancePhase == 3 then
            -- Phase 3: Search for line by turning right
            leftV = speed * 0.4
            rightV = -speed * 0.4
            if middle or left or right then
                avoidingObstacle = false
                avoidancePhase = 0
                print(">>> Line found! Resuming normal operation.")
            elseif phaseTime > 2.0 then
                -- Reset and try again
                avoidancePhase = 2
                avoidanceStartTime = currentTime
            end
        end
    else
        -- Normal line following with RIGHT priority
        if middle then
            if right then
                leftV = speed
                rightV = speed * 0.1
            elseif left then
                leftV = speed
                rightV = speed * 0.8
            else
                leftV = speed
                rightV = speed
            end
        elseif right then
            leftV = speed
            rightV = speed * 0.0
        elseif left then
            leftV = speed * 0.0
            rightV = speed
        else
            leftV = speed * 0.4
            rightV = -speed * 0.4
        end
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
