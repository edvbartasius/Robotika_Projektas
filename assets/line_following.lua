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
    minMaxSpeed = { 50 * math.pi / 180, 300 * math.pi / 180 }
    floorSensorHandles = { -1, -1, -1 }
    floorSensorHandles[1] = sim.getObject("./leftSensor")
    floorSensorHandles[2] = sim.getObject("./middleSensor")
    floorSensorHandles[3] = sim.getObject("./rightSensor")
    robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, { 1, 1, 0 }, nil, nil,
        { 1, 1, 0 })


    --[=====[
 ######     #    ######     #    #     # ####### ####### ####### ######   #####
 #     #   # #   #     #   # #   ##   ## #          #    #       #     # #     #
 #     #  #   #  #     #  #   #  # # # # #          #    #       #     # #
 ######  #     # ######  #     # #  #  # #####      #    #####   ######   #####
 #       ####### #   #   ####### #     # #          #    #       #   #         #
 #       #     # #    #  #     # #     # #          #    #       #    #  #     #
 #       #     # #     # #     # #     # #######    #    ####### #     #  #####

    --]=====]
    -- Robot control parameters
    ignoreUntilTime = -1
    currentTime = 0
    lastTurnTime = 0
    turnDirection = nil -- Will be 'left' or 'right'

    -- Robot geometry parameters
    wheelRadius = 0.04
    bodyDistanceToWheel = 0.1
    turnSpeed = 0.05

    -- Precompute turn parameters
    local cellSize = 2
    local wL = -turnSpeed / wheelRadius
    local wR = turnSpeed / wheelRadius
    local minTurnTime = (bodyDistanceToWheel * math.pi / 2) / turnSpeed
    precomputedTurnL = wL
    precomputedTurnR = wR
    precomputedTurnDuration = minTurnTime

    print("Turn parameters - wL: " .. wL .. ", wR: " .. wR .. ", duration: " .. minTurnTime .. "s")


    --[=====[
 #######  #####  #     #
 #       #     # ##   ##
 #       #       # # # #
 #####    #####  #  #  #
 #             # #     #
 #       #     # #     #
 #        #####  #     #

    --]=====]
    fsm = machine.create({
        initial = 'following',
        events = {
            { name = 'detect_both',   from = 'following', to = 'turning' },
            { name = 'turn_complete', from = 'turning',   to = 'ignoring' },
            { name = 'timer_expired', from = 'ignoring',  to = 'following' }
        },
        callbacks = {
            onenterturning = function()
                print(">>> Transition to TURNING - Direction: " .. turnDirection)
                lastTurnTime = currentTime
            end,
            onenterignoring = function()
                print(">>> Transition to IGNORING (turn complete)")
                ignoreUntilTime = currentTime + 1
            end,
            onenterfollowing = function()
                print(">>> Transition to FOLLOWING (timer expired)")
                turnDirection = nil
            end
        }
    })

    --[=====[
 #     # ###
 #     #  #
 #     #  #
 #     #  #
 #     #  #
 #     #  #
  #####  ###

    --]=====]
    xml = '<ui title="' ..
    sim.getObjectAlias(bubbleRobBase, 1) .. ' speed" closeable="false" resizeable="false" activate="false">' .. [[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui = simUI.create(xml)
    speed = (minMaxSpeed[1] + minMaxSpeed[2]) * 0.5
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
    currentTime = sim.getSimulationTime()
    result = sim.readProximitySensor(noseSensor)

    sensorReading = { false, false, false }
    for i = 1, 3, 1 do
        result, data = sim.readVisionSensor(floorSensorHandles[i])
        if (result >= 0) then
            sensorReading[i] = (data[11] < 0.5) -- data[11] is the average of intensity of the image
        end
    end

    --[=====[
  #####  #######    #    ####### #######    ####### ######     #    #     #  #####  ### ####### ### ####### #     #  #####
 #     #    #      # #      #    #             #    #     #   # #   ##    # #     #  #     #     #  #     # ##    # #     #
 #          #     #   #     #    #             #    #     #  #   #  # #   # #        #     #     #  #     # # #   # #
  #####     #    #     #    #    #####         #    ######  #     # #  #  #  #####   #     #     #  #     # #  #  #  #####
       #    #    #######    #    #             #    #   #   ####### #   # #       #  #     #     #  #     # #   # #       #
 #     #    #    #     #    #    #             #    #    #  #     # #    ## #     #  #     #     #  #     # #    ## #     #
  #####     #    #     #    #    #######       #    #     # #     # #     #  #####  ###    #    ### ####### #     #  #####

    --]=====]
    if fsm:is('following') then
        if sensorReading[1] and not sensorReading[3] then
            turnDirection = 'right'
            fsm:detect_both()
        elseif sensorReading[3] and not sensorReading[1] then
            turnDirection = 'left'
            fsm:detect_both()
        elseif sensorReading[1] and sensorReading[3] then
            turnDirection = 'right'
            fsm:detect_both()
        end
    elseif fsm:is('turning') then
        local timeElapsed = currentTime - lastTurnTime
        print(string.format("[TURNING %s] Elapsed: %.3f / Duration: %.3f (%.1f%%)", string.upper(turnDirection),
            timeElapsed, precomputedTurnDuration, (timeElapsed / precomputedTurnDuration * 100)))
        if timeElapsed > precomputedTurnDuration then
            print(">>> Turn complete!")
            fsm:turn_complete()
        end
    elseif fsm:is('ignoring') then
        if ignoreUntilTime <= currentTime then
            fsm:timer_expired()
        else
            print(">>> Ignoring sensors for " .. string.format("%.1f", ignoreUntilTime - currentTime) .. " more seconds")
        end
    end



    --[=====[
    #     #####  #######
   # #   #     #    #
  #   #  #          #
 #     # #          #
 ####### #          #
 #     # #     #    #
 #     #  #####     #

    --]=====]
    local leftV = 0
    local rightV = 0

    if fsm:is('following') then
        print("[FOLLOWING] Sensors: L=" ..
        tostring(sensorReading[1]) .. " M=" .. tostring(sensorReading[2]) .. " R=" .. tostring(sensorReading[3]))

        leftV = speed
        rightV = speed

        if sensorReading[1] and not sensorReading[3] then
            leftV = speed * 0.5
            print("  - Line on LEFT: slowing left motor (turn RIGHT)")
        end
        if sensorReading[3] and not sensorReading[1] then
            rightV = speed * 0.5
            print("  - Line on RIGHT: slowing right motor (turn LEFT)")
        end
    elseif fsm:is('turning') then
        print("[TURNING] Performing 90° turn - Direction: " .. string.upper(turnDirection))
        if turnDirection == 'left' then
            leftV = precomputedTurnL
            rightV = precomputedTurnR
            print("  - Left turn (R forward, L backward)")
        else -- turnDirection == 'right'
            leftV = precomputedTurnR
            rightV = precomputedTurnL
            print("  - Right turn (L forward, R backward)")
        end
    elseif fsm:is('ignoring') then
        print("[IGNORING] Moving forward at full speed")
        leftV = speed
        rightV = speed
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
