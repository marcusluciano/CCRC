--- @ardupilot Strobe light via PWM relay with R/C channel on-off switch
---   *** Strobe turns on automatically when motors are armed ***
--- @author marcus.luciano@gmail.com
--- @author https://www.chestercountyrc.com/
--- @license WTFPL

--- @use
---   - Assign the strobe light relay's servo number to SCR_USER1
--- 
---   - If you want to use an R/C on-off switch, assign the on-off R/C channel to
---     SCR_USER2, otherwise set SCR_USER2 to 0 (zero).

local STROBE_SERVO = param:get('SCR_USER1') - 1 -- Servo number is zero indexed

if STROBE_SERVO < 0 or STROBE_SERVO > 11 then
    gcs:send_text(1, "strobe.lua: invalid SCR_USER1 servo# (1-12)")
    return
end

local RC_ON_OFF = param:get('SCR_USER2') -- Set to 0 to disable manual R/C on-off switch

if RC_ON_OFF > 32 then
    RC_ON_OFF = 0
end

local ON_OFF_arr = {} -- Relay PWM low, high i.e. off and on pwm fences
ON_OFF_arr[0] = 1100 -- Off pwm value
ON_OFF_arr[1] = 1900 -- On pwm value

local SWITCH_arr = { -- These are ON_OFF_arr index values, each 1/10th second
    1, 1,
    0,
    1, 1, 1, 1,
    0, 0, 0, 0, 0, 0
}

local switch_index = 0 -- Our position in SWITCH_arr

local MS_DELAY = 100 -- SWITCH_arr step interval

local LONG_DELAY = 1000 -- If strobe is off, only check once per LONG_DELAY ms

local rc_input = -1

function update()

    if RC_ON_OFF > 0 then --- Check the manual switch value
        rc_input = rc:get_pwm(RC_ON_OFF)
    end

    if arming:is_armed() or rc_input > 1500  then --- Strobe is running

        switch_index = switch_index + 1

        if switch_index > #SWITCH_arr then
            switch_index = 1
        end

        SRV_Channels:set_output_pwm_chan_timeout(STROBE_SERVO, ON_OFF_arr[ SWITCH_arr[switch_index] ], MS_DELAY)

        return update, MS_DELAY

    else --- Strobe is off

        SRV_Channels:set_output_pwm_chan_timeout(STROBE_SERVO, ON_OFF_arr[ 0 ], LONG_DELAY)

        return update, LONG_DELAY

    end

end

gcs:send_text(1, "strobe.lua started")

return update, MS_DELAY