import time

def raspimouse(v_ok, v_servo, v_esc, v_fps, speed_max):
	print('raspimouse start')

	filename_motoren = "/dev/rtmotoren0"
	filename_motor_r = "/dev/rtmotor_raw_r0"
	filename_motor_l = "/dev/rtmotor_raw_l0"
	filename_motor = "/dev/rtmotor0"

	def motor_drive(freq_l="0", freq_r="0"):
		with open(filename_motor_l, 'w') as f:
			f.write(freq_l)
		with open(filename_motor_r, 'w') as f:
			f.write(freq_r)

	with open(filename_motoren, 'w') as f:
		f.write("1")

	while v_ok.value:
		pulse_L = v_esc.value * (1 - v_servo.value * 0.25)
		pulse_R = v_esc.value * (1 + v_servo.value * 0.25)

		if pulse_L > 1:
			pulse_L = 1
		if pulse_L < -1:
			pulse_L = -1

		if pulse_R > 1:
			pulse_R = 1
		if pulse_R < -1:
			pulse_R = -1

		motor_drive(str(round(speed_max * pulse_R)), str(round(speed_max * pulse_L)))

		time.sleep(1.0 / v_fps.value)

	with open(filename_motoren, 'w') as f:
		f.write("0")

	print('raspimouse end')
