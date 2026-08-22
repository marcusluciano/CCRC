--- @ardupilot HLK-LD2451 24GHz Doppler radar rangefinder driver
--- @use For use as a radar altimeter.  Farthest target is always selected.
--- @author marcus.luciano@gmail.com
--- @author https://www.chestercountyrc.com/
--- @license MIT

--- @type integer - MAV_SEVERITY_CRITICAL
local MSG_STAT = 2

--- @type integer - Scripting type rangefinder instance #
local rangefinder_number = 0

local rangefinder_instance = assert(rangefinder:get_backend(rangefinder_number), 
    gcs:send_text(MSG_STAT, "Lua rangefinder instance not found"))

local rngfnd_x = string.format("RNGFND%x", rangefinder_number)
param:set_by_name(rngfnd_x + '_MIN', 50) -- cm
param:set_by_name(rngfnd_x + '_MAX', 9000) -- cm

local serial_port = assert(serial:find_serial(0), 
    gcs:send_text(MSG_STAT, "Serial port type 28 not found")) -- First LUA port

serial_port:begin(115200)
serial_port:configure_parity(0)
serial_port:set_stop_bits(1)

local MSG_HEADER = {0xF4, 0xF3, 0xF2, 0xF1}

local MSG_TRAILER = {0xF8, 0xF7, 0xF6, 0xF5}

local CONFIG_HEADER = {0xFD, 0xFC, 0xFB, 0xFA}

local CONFIG_TRAILER = {0x04, 0x03, 0x02, 0x01}

--- length + data to put device into config mode 
local CONFIG_MODE = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00} 
--- Length will be tested during ACK read
local CONFIG_ACK = {0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}

--- len=0x06, CW=0x02, 95m max, bidirectional, 1 kph threshold, 0.25s delay
local CONFIG_DATA = {0x06, 0x00, 0x02, 0x00, 0x5F, 0x02, 0x01, 0x40}
--- Length will be tested during ACK read
local TD_STATUS_ACK = {0x02, 0x01, 0x00, 0x00}

--- length + data to get out of config mode 
local CONFIG_END = {0x02, 0x00, 0xFE, 0x00}
--- Length will be tested during ACK read
local CONFIG_END_ACK = {0xFE, 0x01, 0x00, 0x00}

--- 2 data len + 1 Target Qty + 1 Alarm (and 0 targets) 
local MIN_MSG_BYTES = #MSG_HEADER + 4 + #MSG_TRAILER

local MIN_ACK_BYTES = #CONFIG_HEADER + 2 + #CONFIG_TRAILER

--- @type integer - Data length inside reporting frame
local data_length = 0

--- @type integer
local target_quantity = 0

--- @type integer loop index
local index = 0

--- @type boolean
local incoming = false

--- @type integer - Max of all targets
local distance_m = -1

--- @type integer - angle of max target
local target_angle = -1

--- @type integer
local bytes_ready = 0

--- Frame data ---
--- @type integer degrees, +/-10 (0x00 - 0xFF)
local angle = 0x80 -- 0 degrees
--- @type integer meters
local range = 0x00
--- @type integer
local direction = 0x00
--- @type integer
local speed = 0x00
--- @type integer
local snr = 0x00

--- @type integer - Waiting for data loop counter
local ack_count = 0

--- @type integer - 99 MAXIMUM, do not use 100
local MAX_ACKS = 20 -- ms

--- @type integer
local datum = 0x00

local function handle_message()

    target_quantity = serial_port:read()

    incoming = serial_port:read()

    bytes_ready = bytes_ready - 2

    index = 0

    distance_m = -1

    while index < target_quantity do
        -- Find the target that is farthest away (i.e. the ground)
        index = index + 1

        -- Each target has angle, distance, direction, speed, SNR
        angle = serial_port:read()
        range = serial_port:read()
        direction = serial_port:read()
        speed = serial_port:read()
        snr = serial_port:read()

        bytes_ready = bytes_ready - 5

        if range > distance_m and range < 90 and snr > 1 then
            distance_m = range
            target_angle = angle
        end
    end

    -- Set the distance
    if distance_m > 0.5 and distance_m < 90 then
        --- @todo See if we need to compute adjustment 
        --- based on target_angle if banking or pitching  
        rangefinder_instance:distance(distance_m)
    end
end

local function update()
    --- Reporting Frame ---
        -- MSG_HEADER
        -- LenLo LenHi 
        -- Data
        -- MSG_TRAILER

    --- Data ---
        -- Target Qty 1 byte
        -- Incoming? (0/1) 1 byte
        -- 5 bytes per target (angle, range, direction, speed, snr)

    bytes_ready = serial_port:available()

    if bytes_ready < MIN_MSG_BYTES then

        ack_count = ack_count + 1

        assert(ack_count < MAX_ACKS, gcs:send_text(MSG_STAT, "HLK-LD2451 not responding"))

        return update, 1
    end

    while bytes_ready >= MIN_MSG_BYTES do

        if serial_port:read() == MSG_HEADER[1] and -- note that reading stops here on mismatch
            serial_port:read() == MSG_HEADER[2] and -- and this does not get read if [1] failed
            serial_port:read() == MSG_HEADER[3] and -- etc.
            serial_port:read() == MSG_HEADER[4] then

            data_length = serial_port:read() + (256 * serial_port:read())

            bytes_ready = bytes_ready - 6

            handle_message()

            index = 0

            while index < #CONFIG_TRAILER and bytes_ready > 0 do

                index = index + 1

                datum = serial_port.read()

                bytes_ready = bytes_ready - 1

                if datum ~= CONFIG_TRAILER[index] then
                    index = #CONFIG_TRAILER -- Bail on error
                end
            end

        else
            bytes_ready = bytes_ready - 1

            if bytes_ready < 1 then
                ack_count = 0
                return update, math.abs(100 - ack_count) -- We're out of sync, advance the timing
            end
        end
    end

    return update, 100 -- 10 Hz refresh rate
end

---@function read_ack 
---@param ACK_arr table<number>
local function read_ack(ACK_arr)

    data_length = serial_port:read() + (256 * serial_port:read())

    assert(data_length == #ACK_arr + #CONFIG_TRAILER, 
        gcs:send_text(MSG_STAT,"hlk-ld2431 bad ack len"))

    index = 0

    while (index < #ACK_arr) do

        index = index + 1

        assert(serial_port:read() == ACK_arr[index], 
            gcs:send_text(MSG_STAT, string.format("hlk-ld2451 ack[%d] err", index)))

        data_length = data_length - 1
    end

    index = 0

    while (index < #CONFIG_TRAILER) do

        index = index + 1

        assert(serial_port:read() == CONFIG_TRAILER[index], 
            gcs:send_text(MSG_STAT, 
            string.format( "HLK-LD2451 ACK expected %x", CONFIG_TRAILER[index])))
    end
end

---@function handle_ack - Read ACK and call next fn in line
---@param ACK_arr table - Position of status byte in data (0x01=success)
---@param next_fn function
---@return function, integer
local function handle_ack(ACK_arr, next_fn)

    bytes_ready = serial_port:available()

    if bytes_ready < (MIN_ACK_BYTES + #ACK_arr) then

        ack_count=ack_count + 1

        assert(ack_count < MAX_ACKS, gcs:send_text(MSG_STAT,
                "HLK-LD2451 failed to ACK"))

        return handle_ack(ACK_arr, next_fn), 1

    else
        -- Read ACK, then call next function
        if  serial_port:read() == CONFIG_HEADER[1] and -- note that reading stops here on mismatch
            serial_port:read() == CONFIG_HEADER[2] and -- and this does not get read if [1] failed
            serial_port:read() == CONFIG_HEADER[3] and
            serial_port:read() == CONFIG_HEADER[4] then

            pcall(read_ack, ACK_arr)

            return next_fn, 1
        end
    end
    return handle_ack(ACK_arr, next_fn), 1
end

local function end_config_mode()
    for loop_idx = 1, #CONFIG_HEADER do
        serial_port:write(CONFIG_HEADER[loop_idx])
    end
    for loop_idx = 1, #CONFIG_END do
        serial_port:write(CONFIG_END[loop_idx])
    end
    for loop_idx = 1, #CONFIG_TRAILER do
        serial_port:write(CONFIG_TRAILER[loop_idx])
    end

    ack_count = 0

    return handle_ack(CONFIG_END_ACK, update), 1
end

local function set_target_detection()
    for loop_idx = 1, #CONFIG_HEADER do
        serial_port:write(CONFIG_HEADER[loop_idx])
    end
    for loop_idx = 1, #CONFIG_DATA do
        serial_port:write(CONFIG_DATA[loop_idx])
    end
    for loop_idx = 1, #CONFIG_TRAILER do
        serial_port:write(CONFIG_TRAILER[loop_idx])
    end

    ack_count = 0

    return handle_ack(TD_STATUS_ACK, end_config_mode), 1

end

local function set_config_mode()
    for loop_idx = 1, #CONFIG_HEADER do
        serial_port:write(CONFIG_HEADER[loop_idx])
    end
    for loop_idx = 1, #CONFIG_MODE do
        serial_port:write(CONFIG_MODE[loop_idx])
    end
    for loop_idx = 1, #CONFIG_TRAILER do
        serial_port:write(CONFIG_TRAILER[loop_idx])
    end

    ack_count = 0

    return handle_ack(CONFIG_ACK, set_target_detection), 1

end

gcs:send_text(MSG_STAT, 
    string.format("hlk-lkd2451.lua #%d started", rangefinder_number))

return set_config_mode, 1
