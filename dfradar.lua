--- @ardupilot DFRobot 24GHz Doppler radar rangefinder driver
---  *** No spectral data processing, range only ***
--- @author marcus.luciano@gmail.com
--- @author https://www.chestercountyrc.com/
--- @license WTFPL

--- @use Set SCR_USER6 = radar serial port #
--- @debug Set SCR_USER5 = 1 to debug

local serial_port = find_serial(param:get('SCR_USER6'))

if serial_port == nil then
    gcs:send_text(0, "Radar serial port not found, SCR_USER6=" + param:get('SCR_USER6'))
    return
end

--- @todo Add rangefinder instance, set distance there

local debug_flag = param:get('SCR_USER5') == 1

local bytes_ready = 0

local header_byte = "xFF" -- Packet header is xFFFFFF

local byte_of_data = -1 -- This is our read buffer

local counter = 0 -- Header xFF counter

local distance = -1 -- Centimeters

local k = 0 -- GCS display limiter

serial_port:begin(115200)
serial_port:configure_parity(0)
serial_port:set_stop_bits(1)

function update()

    bytes_ready = serial_port:available()

    counter = 0

    k = k + 1

    while bytes_ready > 0 do

        byte_of_data = serial_port:read()

        bytes_ready = bytes_ready - 1

        if byte_of_data == header_byte then

            counter = counter + 1

            if counter == 3 then -- we found &HFFFFFF

                if bytes_ready >= 2 then

                    distance = serial_port:read() * 256 + serial_port:read()

                    -- Output telemetry debug if SCR_USER5 == 1
                    if k >= 20 then -- 20 == 2 seconds
                        if debug_flag then
                            gcs:send_text(0, string.format("Radar distance: %d cm",
                                distance))
                        end

                        k = 0
                    end

                    if distance > 49 and distance < 2001 then

                        location:terrain_alt(1) -- Set relative to terrain

                        location:alt(distance) -- Set FC altitude

                    end

                    counter = 0
                end
            end
        else
            counter = 0
        end
    end

    return update, 100 -- This radar runs at a 10 Hz refresh rate
end

gcs:send_text(1, "dfradar.lua started")

return update, MS_DELAY