import struct
import time

def dualshock4(v_ok, v_wheel, v_trigger, v_esc_on, v_auto_on, v_log_on, v_log_print_on, v_angle_log_on, count_reset, v_servo_trim, v_speed_trim):
	print('dualshock4 start')

	js = open('/dev/input/js0', 'rb')

	while v_ok.value:
		readbuf = js.read(8)
		if readbuf:
			time, value, vtype = struct.unpack('IhH', readbuf)
			#print("vtype : " + str(vtype))
			#if vtype == 2 or vtype == 1026:
			#	print(time, vtype, value, value / 0x7fff)
			if vtype == 2:
				v_wheel.value = value / 0x7fff
			elif vtype == 1026:
				v_trigger.value = value / 0x7fff * -1
			elif vtype == 1538:
				if value > 0:
					v_servo_trim.value = round(v_servo_trim.value + 0.01, 2)
					if v_servo_trim.value > 1:
						v_servo_trim.value = 1
					print('servo trim :', "{:.2f}".format(v_servo_trim.value))
				if value < 0:
					v_servo_trim.value = round(v_servo_trim.value - 0.01, 2)
					if v_servo_trim.value < -1:
						v_servo_trim.value = -1
					print('servo trim :', "{:.2f}".format(v_servo_trim.value))
			elif vtype == 1794:
				if value < 0:
					v_speed_trim.value = round(v_speed_trim.value + 0.01, 2)
					if v_speed_trim.value > 1:
						v_speed_trim.value = 1
					print('speed trim trim :', "{:.2f}".format(v_speed_trim.value))
				if value > 0:
					v_speed_trim.value = round(v_speed_trim.value - 0.01, 2)
					if v_speed_trim.value < 0:
						v_speed_trim.value = 0
					print('speed trim trim :', "{:.2f}".format(v_speed_trim.value))
			elif vtype == 1 and value == 1:
				v_esc_on.value = not v_esc_on.value
				print('ESC', 'ON' if v_esc_on.value else 'OFF')
			elif vtype == 513 and value == 1:
				v_auto_on.value = not v_auto_on.value
				print('AUTO', 'ON' if v_auto_on.value else 'OFF')
			elif vtype == 257 and value == 1:
				v_log_on.value = not v_log_on.value
				print('LOG', 'ON' if v_log_on.value else 'OFF')
			elif vtype == 769 and value == 1:
				count_reset.value = not count_reset.value
				print('COUNT_RESET')
			elif vtype == 1281 and value == 1:
				v_log_print_on.value = not v_log_print_on.value
				print('ENCORDER_LOG', 'ON' if v_log_print_on.value else 'OFF')
			elif vtype == 1025 and value == 1:
				v_angle_log_on.value = not v_angle_log_on.value
				print('ANGLE_LOG', 'ON' if v_angle_log_on.value else 'OFF')
	print('dualshock4 end')
