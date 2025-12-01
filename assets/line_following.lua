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
    
    -- Timing
    phaseStartTime = 0
    
    -- State machine
    fsm = machine.create({
        initial = 'following',
        events = {
            { name = 'obstacle_detected', from = 'following',    to = 'turning_away' },
            { name = 'front_clear',       from = 'turning_away', to = 'wall_follow' },
            { name = 'line_found',        from = 'wall_follow',  to = 'following' },
            { name = 'timeout',           from = 'wall_follow',  to = 'searching' },
            { name = 'line_found',        from = 'searching',    to = 'following' },
            { name = 'search_timeout',    from = 'searching',    to = 'wall_follow' }
        },
        callbacks = {
            onturning_away = function() 
                phaseStartTime = sim.getSimulationTime()
                print(">>> STATE: Turning away from obstacle") 
            end,
            onwall_follow = function() 
                phaseStartTime = sim.getSimulationTime()
                print(">>> STATE: Wall following") 
            end,
            onsearching = function() 
                phaseStartTime = sim.getSimulationTime()
                print(">>> STATE: Searching for line") 
            end,
            onfollowing = function() 
                print(">>> STATE: Following line") 
            end
        }
    })
    
--[=====[
  #####  ######     #    ######  #     #
 #     # #     #   # #   #     # #     #
 #       #     #  #   #  #     # #     #
 #  #### ######  #     # ######  #######
 #     # #   #   ####### #       #     #
 #     # #    #  #     # #       #     #
  #####  #     # #     # #       #     #

--]=====]
    xml = '<ui title="Speed" closeable="false" resizeable="false" activate="false">' .. [[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui = simUI.create(xml)
    simUI.setSliderValue(ui, 1, 100 * (speed - minMaxSpeed[1]) / (minMaxSpeed[2] - minMaxSpeed[1]))
end


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
    local phaseTime = currentTime - phaseStartTime
        
    -- Read proximity sensors
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
    
    local lineDetected = middle or left or right
    local leftV, rightV = 0, 0  -- Default to stopped
    
    -------------------------
    -- STATE: FOLLOWING
    -------------------------
    if fsm:is('following') then
        if obstacleDetected then
            fsm:obstacle_detected()
            -- Set motor values for transition frame
            leftV, rightV = -speed * 0.5, speed * 0.5
        else
            -- Line following with RIGHT priority
            if middle then
                if right then
                    leftV, rightV = speed, speed * 0.1
                elseif left then
                    leftV, rightV = speed, speed * 0.8
                else
                    leftV, rightV = speed, speed
                end
            elseif right then
                leftV, rightV = speed, 0
            elseif left then
                leftV, rightV = 0, speed
            else
                -- Line lost - spin right
                leftV, rightV = speed * 0.4, -speed * 0.4
            end
        end
    
    -------------------------
    -- STATE: TURNING AWAY
    -------------------------
    elseif fsm:is('turning_away') then
        -- Turn left until front is clear
        leftV = -speed * 0.5
        rightV = speed * 0.5
        
        if not noseDetected and not frontRightDetected and phaseTime > 0.3 then
            fsm:front_clear()
        end
    
    -------------------------
    -- STATE: WALL FOLLOW
    -------------------------
    elseif fsm:is('wall_follow') then
        if frontRightDetected or noseDetected then
            -- Obstacle on right or ahead - turn left
            leftV, rightV = speed * 0.3, speed * 0.6
        elseif frontLeftDetected then
            -- Obstacle on left - turn right
            leftV, rightV = speed * 0.6, speed * 0.3
        else
            -- Clear - go forward with slight right drift
            leftV, rightV = speed * 0.7, speed * 0.5
        end
        
        -- Transitions
        if lineDetected and not obstacleDetected then
            fsm:line_found()
        elseif phaseTime > 3.0 then
            fsm:timeout()
        end
    
    -------------------------
    -- STATE: SEARCHING
    -------------------------
    elseif fsm:is('searching') then
        -- Spin right to find line
        leftV = speed * 0.4
        rightV = -speed * 0.4
        
        if lineDetected then
            fsm:line_found()
        elseif phaseTime > 2.0 then
            fsm:search_timeout()
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
