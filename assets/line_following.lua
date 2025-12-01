-- Load FSM library
local machine = require("statemachine")

function sysCall_init()
    bubbleRobBase=sim.getObject('.')
    leftMotor=sim.getObject("./leftMotor")
    rightMotor=sim.getObject("./rightMotor")
    noseSensor=sim.getObject("./sensingNose")
    minMaxSpeed={50*math.pi/180,300*math.pi/180}
    floorSensorHandles={-1,-1,-1}
    floorSensorHandles[1]=sim.getObject("./leftSensor")
    floorSensorHandles[2]=sim.getObject("./middleSensor")
    floorSensorHandles[3]=sim.getObject("./rightSensor")
    robotTrace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
    
    ignoreUntilTime = -1
    currentTime = 0
    
    -- Create FSM
    fsm = machine.create({
        initial = 'following',
        events = {
            { name = 'detect_both', from = 'following', to = 'ignoring' },
            { name = 'timer_expired', from = 'ignoring', to = 'following' }
        },
        callbacks = {
            onenterignoring = function()
                print(">>> Transition to IGNORING (both sensors detected)")
                ignoreUntilTime = currentTime + 5
            end,
            onenterfollowing = function()
                print(">>> Transition to FOLLOWING (timer expired)")
            end
        }
    })
    
    -- Create the custom UI:
    xml = '<ui title="'..sim.getObjectAlias(bubbleRobBase,1)..' speed" closeable="false" resizeable="false" activate="false">'..[[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui=simUI.create(xml)
    speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
    simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))
    
end

function sysCall_sensing()
    local p=sim.getObjectPosition(bubbleRobBase,-1)
    sim.addDrawingObjectItem(robotTrace,p)
end 

function speedChange_callback(ui,id,newVal)
    speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
end

function sysCall_actuation() 
    currentTime = sim.getSimulationTime()
    result=sim.readProximitySensor(noseSensor)
    
    -- read the line detection sensors:
    sensorReading={false,false,false}
    for i=1,3,1 do
        result,data=sim.readVisionSensor(floorSensorHandles[i])
        if (result>=0) then
            sensorReading[i]=(data[11]<0.5) -- data[11] is the average of intensity of the image
        end
    end

    -- FSM Logic - State transitions
    if fsm:is('following') then
        if sensorReading[1] and sensorReading[3] then
            -- Both left and right sensors detect line
            fsm:detect_both()
        end
    elseif fsm:is('ignoring') then
        if ignoreUntilTime <= currentTime then
            fsm:timer_expired()
        else
            print(">>> Ignoring sensors for " .. string.format("%.1f", ignoreUntilTime - currentTime) .. " more seconds")
        end
    end
    
    -- Initialize motor velocities
    local leftV = 0
    local rightV = 0
    
    -- State actions
    if fsm:is('following') then
        print("[FOLLOWING] Sensors: L=" .. tostring(sensorReading[1]) .. " M=" .. tostring(sensorReading[2]) .. " R=" .. tostring(sensorReading[3]))
        
        -- Line following control
        leftV = speed
        rightV = speed
        
        if sensorReading[1] then
            leftV = speed * 0.5
            print("  - Slowing left motor")
        end
        if sensorReading[3] then
            rightV = speed * 0.5
            print("  - Slowing right motor")
        end
        
    elseif fsm:is('ignoring') then
        print("[IGNORING] Moving forward at full speed")
        leftV = speed
        rightV = speed
    end

    -- Apply motor velocities
    sim.setJointTargetVelocity(leftMotor,leftV)
    sim.setJointTargetVelocity(rightMotor,rightV)
end 

function sysCall_cleanup() 
    simUI.destroy(ui)
end 
