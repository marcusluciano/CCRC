--- @ardupilot HLK-LD2451 24GHz Doppler radar rangefinder driver
--- @use For use as a radar altimeter.  Farthest target is always selected.
--- @author marcus.luciano@gmail.com
--- @author https://www.chestercountyrc.com/
--- @license WTFPL

local serial_port = assert(serial:find_serial(0), 
    "Serial port type 28 not found") -- First LUA port

local rangefinder_number = 0

local rangefinder_instance = assert(rangefinder:get_backend(rangefinder_number), 
    "Lua rangefinder instance not found")

local FRAME_HEADER = {0xF4, 0xF3, 0xF2, 0xF1}

local FRAME_TRAILER = {0xF8, 0xF7, 0xF6, 0xF5}

local CONFIG_MODE = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01}

local CONFIG_END = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01}

local trailer = {-1, -1, -1, -1}

local data_length = 0 -- Data length inside frame

local target_quantity = 0

local target_index = 0

local distance_m = -1

local bytes_ready = 0

--local angle = 0x80

local range = 0

--local direction = 0

--local speed = 0

--local snr = 0

local ack_count = 0

local MAX_ACKS = 10 -- ms

local loop_idx = 0

---@param rangefinder_num number
---@param min_cm number
---@param max_cm number
local function rangefinder_limits_set(rangefinder_num, min_cm, max_cm)

    local rngfnd_x = string.format("RNGFND%x", rangefinder_num) --- hex

    param:set_by_name(rngfnd_x + '_MIN', min_cm)
    param:set_by_name(rngfnd_x + '_MAX', max_cm)
end

rangefinder_limits_set(rangefinder_number, 50, 9000)

serial_port:begin(115200)
serial_port:configure_parity(0)
serial_port:set_stop_bits(1)

function update()

    bytes_ready = serial_port:available()

    while bytes_ready > 0 do

        if serial_port:read() == FRAME_HEADER(1) and
            serial_port:read() == FRAME_HEADER(2) and
            serial_port:read() == FRAME_HEADER(3) and
            serial_port:read() == FRAME_HEADER(4) then

            data_length = serial_port:read() + (256 * serial_port:read())

            target_quantity = serial_port:read()

            serial_port:read() -- Discard alarm information

            bytes_ready = bytes_ready - 8

            if target_quantity * 5 + 2 > data_length then
                -- Frame contains errors
                while bytes_ready > 0 do
                    serial_port:read()
                    bytes_ready = bytes_ready - 1
                end
                return update, 100
            end

            target_index = 0

            distance_m = -1

            while target_index < target_quantity do
                -- Find the target that is farthest away (i.e. the ground)
                target_index = target_index + 1

                -- Read angle, distance, direction, speed, SNR
                serial_port:read() -- angle = 
                range = serial_port:read()
                serial_port:read() -- direction = 
                serial_port:read() -- speed = 
                serial_port:read() -- snr = 

                bytes_ready = bytes_ready - 5

                if range > distance_m then
                    distance_m = range
                end
            end

            -- Set the distance
            if distance_m > 0.5 and distance_m < 90 then
                rangefinder_instance:distance(distance_m)
            end

            -- Read trailer
            if trailer[1] = serial_port.read() then
                if trailer[2] = serial_port.read() then
                    if trailer[3] = serial_port.read() then
                        if trailer[4] = serial_port.read() then
                            if trailer[4] ~= 0xF5 then
                                -- gcs:send_text(0, "Packet error on port type 28 instance 1")
                            end
                        end
                        bytes_ready = bytes_ready - 1
                    end
                    bytes_ready = bytes_ready - 1
                end
                bytes_ready = bytes_ready - 1
            end
            bytes_ready = bytes_ready - 1

        else
            bytes_ready = bytes_ready - 1

            if bytes_ready < 1 then
                return update, 100
            end
        end
    end

    return update, 100 -- This radar runs at a 10 Hz refresh rate
end

function program_ack()

    bytes_ready = serial_port:available()

    if bytes_ready < 18 then

        ack_count=ack_count + 1
        if ack_count > MAX_ACKS then
            gcs:send_text(3,
            "Rangefinder failed to enter programming mode")
            return
        end
    else
        -- Confirm ACK, then call device programming

    end
end

function program_mode()

    for loop_idx = 1, #CONFIG_MODE do
        serial_port:write(CONFIG_MODE[loop_idx])
    end

    ack_count = 0

    return program_ack, 1
end

gcs:send_text(1, "hlk-lkd2451.lua started")

return program_mode, 1

