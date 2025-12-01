-- Finite State Machine for Line Following (Test Version)

-- States
STATE = {
    FOLLOWING = 1,
    IGNORING = 2
}

-- State names for printing
STATE_NAME = {
    [STATE.FOLLOWING] = "FOLLOWING",
    [STATE.IGNORING] = "IGNORING"
}

-- Initialize FSM
local state = STATE.FOLLOWING
local ignoreUntilTime = -1
local currentTime = 0

-- Simulated sensor readings
local function readSensors()
    return {
        left = math.random(0, 1) == 1,
        middle = math.random(0, 1) == 1,
        right = math.random(0, 1) == 1
    }
end

-- Main FSM loop
local function updateFSM()
    local sensors = readSensors()
    
    -- Print current state
    print("\n[TIME: " .. currentTime .. "] State: " .. STATE_NAME[state])
    print("Sensors - Left: " .. tostring(sensors.left) .. ", Middle: " .. tostring(sensors.middle) .. ", Right: " .. tostring(sensors.right))
    
    -- State transitions
    if state == STATE.FOLLOWING then
        if sensors.left and sensors.right then
            state = STATE.IGNORING
            ignoreUntilTime = currentTime + 5
            print(">>> Transition to IGNORING (both sensors detected)")
        end
    elseif state == STATE.IGNORING then
        if ignoreUntilTime <= currentTime then
            state = STATE.FOLLOWING
            print(">>> Transition to FOLLOWING (timer expired)")
        else
            print(">>> Ignoring sensors for " .. (ignoreUntilTime - currentTime) .. " more ticks")
        end
    end
    
    -- State actions
    if state == STATE.FOLLOWING then
        print(">>> ACTION: Line following correction")
        if sensors.left then
            print("    - Slow left motor")
        end
        if sensors.right then
            print("    - Slow right motor")
        end
    elseif state == STATE.IGNORING then
        print(">>> ACTION: Move forward at full speed (ignoring sensors)")
    end
end

-- Simulation loop
print("=== FSM Line Following Test ===")
for i = 1, 20 do
    currentTime = i
    updateFSM()
end