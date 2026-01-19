-- Used for dual motor fixed wing without aileron control

local K_RUDDER = 21
local K_THROTTLE = 70
local K_SCRIPTING1 = 94
local K_SCRIPTING2 = 95 
local servo1_chan = SRV_Channels:find_channel(K_SCRIPTING1)
local servo2_chan = SRV_Channels:find_channel(K_SCRIPTING2)

local PARAM_TABLE_KEY = 72
assert(param:add_table(PARAM_TABLE_KEY, "DIFF_", 30), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1,  'SCALE', 1), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2,  'SERVO1_REV', 0), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 3,  'SERVO2_REV', 0), 'could not add param1')
local SCALE = Parameter()
local SERVO1_REV = Parameter()
local SERVO2_REV = Parameter()
SCALE:init('DIFF_SCALE')
SERVO1_REV:init('DIFF_SERVO1_REV')
SERVO2_REV:init('DIFF_SERVO2_REV')

function update()
	local scale = SCALE:get()
	local servo1_reverse = SERVO1_REV:get()
	local servo2_reverse = SERVO2_REV:get()
	rudder_pwm = SRV_Channels:get_output_pwm(K_RUDDER)
	throttle_pwm = SRV_Channels:get_output_pwm(K_THROTTLE)
    if arming:is_armed() then
		cal_num = (1500 - rudder_pwm)*scale
		cal_num = math.ceil(cal_num)
		--if servo1_reverse == 1 then
	        servo1_pwm = (servo1_reverse==1) and (throttle_pwm + cal_num) or (throttle_pwm - cal_num)
		--else
			--servo1_pwm = throttle_pwm - cal_num
		--end
		
		--if servo2_reverse == 1 then
			servo2_pwm = (servo2_reverse==1) and (throttle_pwm - cal_num) or (throttle_pwm + cal_num)
		--else
			--servo2_pwm = throttle_pwm + cal_num
		--end
		
		if servo1_pwm < 1000 then
			servo1_pwm = 1000
		elseif servo1_pwm > 2000 then
			servo1_pwm = 2000
		end
		
		if servo2_pwm < 1000 then
			servo2_pwm = 1000
		elseif servo2_pwm > 2000 then
			servo2_pwm = 2000
		end
			
		SRV_Channels:set_output_pwm_chan(servo1_chan, servo1_pwm)
		SRV_Channels:set_output_pwm_chan(servo2_chan, servo2_pwm)
        --gcs:send_text(6, "arm")
    else
		SRV_Channels:set_output_pwm_chan(servo1_chan, 1000)
		SRV_Channels:set_output_pwm_chan(servo2_chan, 1000)
        --gcs:send_text(6, "disarm")
    end	
    return update, 10
end

gcs:send_text(6, "servo_set_get.lua is running")

return update()
