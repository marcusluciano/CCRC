--- @ardupilot HLK-LD2451 24GHz Doppler radar rangefinder driver
--- @use For use as a radar altimeter.  Farthest target is always selected.
--- @author marcus.luciano@gmail.com
--- @author https://www.chestercountyrc.com/
--- @license MIT

--- @type integer - Scripting type rangefinder instance #
local rangefinder_number = 0

local rangefinder_instance = assert(rangefinder:get_backend(rangefinder_number), 
    gcs:send_text(0, "Lua rangefinder instance not found"))

local rngfnd_x = string.format("RNGFND%x", rangefinder_number)
param:set_by_name(rngfnd_x + '_MIN', 50) -- cm
param:set_by_name(rngfnd_x + '_MAX', 9000) -- cm

local serial_port = assert(serial:find_serial(0), 
    gcs:send_text(0, "Serial port type 28 not found")) -- First LUA port

serial_port:begin(115200)
serial_port:configure_parity(0)
serial_port:set_stop_bits(1)

local MSG_HEADER = {0xF4, 0xF3, 0xF2, 0xF1}

local MSG_TRAILER = {0xF8, 0xF7, 0xF6, 0xF5}

--- 2 data len + 1 Target Qty + 1 Alarm (and 0 targets) 
local MIN_MSG_BYTES = #MSG_HEADER + 4 + #MSG_TRAILER

local CONFIG_HEADER = {0xFD, 0xFC, 0xFB, 0xFA}

local CONFIG_TRAILER = {0x04, 0x03, 0x02, 0x01}

local CONFIG_MODE = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00} -- + CMD hdr/trlr

local CONFIG_STATUS_INDEX = 5 -- 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00

local SET_TD_STATUS_INDEX = 2 -- 0x02, 0x01, 0x00, 0x00

--- 95m max, bidirectional, 1 kph threshold, 0.25s delay
local CONFIGURATION = {0x5F, 0x02, 0x01, 0x40}

local END_CONFIG = {0x02, 0x00, 0xFE, 0x00}

--local END_CONFIG_ACK = {0x04, 0x00, 0xFE, 0x01, 0x00, 0x00}

local MIN_ACK_LEN = #CONFIG_HEADER + 2 + 4 + #CONFIG_TRAILER

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
local MAX_ACKS = 50 -- ms

--- @type integer
local status_byte = 0x00

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

        assert(ack_count < MAX_ACKS, gcs:send_text(0, "HLK-LD2451 not responding"))

        return update, 1
    end

    while bytes_ready >= MIN_MSG_BYTES do

        if serial_port:read() == MSG_HEADER[1] and -- note that reading stops here on mismatch
            serial_port:read() == MSG_HEADER[2] and -- and this does not get read if [1] failed
            serial_port:read() == MSG_HEADER[3] and
            serial_port:read() == MSG_HEADER[4] then

            data_length = serial_port:read() + (256 * serial_port:read())

            target_quantity = serial_port:read()

            serial_port:read() -- incoming = serial_port:read() == 0x01

            bytes_ready = bytes_ready - 8

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
                --- @todo Compute and return Cos(target_angle) * distance_m??? 
                --- I think AP does this for us
                rangefinder_instance:distance(distance_m)
            end

            -- Read trailer
            if CONFIG_TRAILER[1] == serial_port.read() then
                bytes_ready = bytes_ready - 1
                if CONFIG_TRAILER[2] == serial_port.read() then
                    bytes_ready = bytes_ready - 1
                    if CONFIG_TRAILER[3] == serial_port.read() then
                        bytes_ready = bytes_ready - 1
                        if CONFIG_TRAILER[4] == serial_port.read() then
                            bytes_ready = bytes_ready - 1
                        end
                    end
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

---@function program_ack - Read ACK and call next fn in line
---@param next_fn function
---@param ack_index integer - Position of status byte in data (0=success)
---@return function, integer
local function program_ack(next_fn, ack_index)

    bytes_ready = serial_port:available()

    if bytes_ready < MIN_ACK_LEN then

        ack_count=ack_count + 1

        assert(ack_count < MAX_ACKS, gcs:send_text(3,
                "HLK-LD2451 failed to ACK"))

        return program_ack(next_fn, ack_index), 1

    else
        -- Read ACK, then call next function
        if  serial_port:read() == CONFIG_HEADER[1] and -- note that reading stops here on mismatch
            serial_port:read() == CONFIG_HEADER[2] and -- and this does not get read if [1] failed
            serial_port:read() == CONFIG_HEADER[3] and
            serial_port:read() == CONFIG_HEADER[4] then

            data_length = serial_port:read() + (256 * serial_port:read())

            index = 0 -- 

            while (data_length > 0) do

                status_byte = serial_port:read()

                index = index + 1

                if index == ack_index then

                    assert(status_byte == 0, 
                        gcs:send_text(3,"HLK-LD2451 configuration failed"))
                end
                data_length = data_length - 1
            end

            assert(serial_port:read() == CONFIG_TRAILER[1], 
                gcs:send_text(3,
                string.format( "HLK-LD2451 ACK error %x", CONFIG_TRAILER[1])))
            assert(serial_port:read() == CONFIG_TRAILER[2], 
                gcs:send_text(3,
                string.format( "HLK-LD2451 ACK error %x", CONFIG_TRAILER[2])))
            assert(serial_port:read() == CONFIG_TRAILER[3], 
                gcs:send_text(3,
                string.format( "HLK-LD2451 ACK error %x", CONFIG_TRAILER[3])))
            assert(serial_port:read() == CONFIG_TRAILER[4], 
                gcs:send_text(3,
                string.format( "HLK-LD2451 ACK error %x", CONFIG_TRAILER[4])))

            return next_fn, 1
        end
    end
    return program_ack(next_fn, ack_index), 1
end

local function end_config()
    for loop_idx = 1, #CONFIG_HEADER do
        serial_port:write(CONFIG_HEADER[loop_idx])
    end
    for loop_idx = 1, #END_CONFIG do
        serial_port:write(END_CONFIG[loop_idx])
    end
    for loop_idx = 1, #CONFIG_TRAILER do
        serial_port:write(CONFIG_TRAILER[loop_idx])
    end

    ack_count = 0

    return program_ack(update, -1), 1
end

local function setTargetDetection()
    for loop_idx = 1, #CONFIG_HEADER do
        serial_port:write(CONFIG_HEADER[loop_idx])
    end
    for loop_idx = 1, #CONFIGURATION do
        serial_port:write(CONFIGURATION[loop_idx])
    end
    for loop_idx = 1, #CONFIG_TRAILER do
        serial_port:write(CONFIG_TRAILER[loop_idx])
    end

    ack_count = 0

    return program_ack(end_config, SET_TD_STATUS_INDEX), 1

end

local function command_mode()
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

    return program_ack(setTargetDetection, CONFIG_STATUS_INDEX), 1

end

gcs:send_text(1, 
    string.format("hlk-lkd2451.lua #%d started", rangefinder_number))

return command_mode, 1
