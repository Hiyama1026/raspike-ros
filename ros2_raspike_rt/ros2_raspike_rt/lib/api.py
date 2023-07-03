from concurrent.futures import ThreadPoolExecutor 
import threading

from .spike_val import *

class motorClass:
    def __init__(self):
        self.lock = threading.Lock() # Lockモジュール生成

    # motor set speed
    def set_wheel_speed(self, wheel_left_speed, wheel_right_speed):
        self.lock.acquire() # ロック
        send_data.set_right_stop_brake_val(0)
        send_data.set_left_stop_brake_val(0)
        send_data.set_left_speed_val(wheel_left_speed)
        send_data.set_right_speed_val(wheel_right_speed)
        self.lock.release() # 解除

    def set_arm_motor_speed(self, arm_speed):
        self.lock.acquire()
        send_data.set_arm_stop_brake_val(0)
        send_data.set_arm_speed_val(arm_speed)
        self.lock.release()

    def set_right_motor_speed(self, right_speed):
        self.lock.acquire()
        send_data.set_right_stop_brake_val(0)
        send_data.set_right_speed_val(right_speed)
        self.lock.release()

    def set_left_motor_speed(self, left_speed):
        self.lock.acquire()
        send_data.set_left_stop_brake_val(0)
        send_data.set_left_speed_val(left_speed)
        self.lock.release()
    
    # motor stop
    def wheel_motor_stop(self):
        self.lock.acquire()
        send_data.set_right_stop_brake_val(1)
        send_data.set_left_stop_brake_val(1)
        self.lock.release()

    def right_motor_stop(self):
        send_data.set_right_stop_brake_val(1)
    
    def left_motor_stop(self):
        send_data.set_left_stop_brake_val(1)

    def arm_motor_stop(self):
        send_data.set_arm_stop_brake_val(1)

    # motor brake
    def wheel_motor_brake(self):
        self.lock.acquire()
        send_data.set_right_stop_brake_val(2)
        send_data.set_left_stop_brake_val(2)
        self.lock.release()
    
    def right_motor_stop(self):
        send_data.set_right_stop_brake_val(2)
    
    def left_motor_stop(self):
        send_data.set_left_stop_brake_val(2)

    def arm_motor_stop(self):
        send_data.set_arm_stop_brake_val(2)

    # motor get count
    def get_right_motor_count(self):
        return rev_data.get_right_count_val()
    def get_left_motor_count(self):
        return rev_data.get_left_count_val()
    def get_arm_motor_count(self):
        return rev_data.get_arm_count_val()

    # motor reset count
    def wheel_motor_reset_count():
        self.lock.acquire()
        send_data.set_right_reset_val(True)
        send_data.set_left_reset_val(True)
        self.lock.release()

    def right_motor_reset_count():
        send_data.set_right_reset_val(True)
    
    def left_motor_reset_count():
        send_data.set_left_reset_val(True)

    def arm_motor_reset_count():
        send_data.set_arm_reset_val(True)
        


motor = motorClass()
    
class colorSensorClass:
    def __init__(self):
        rgb = [0, 0, 0]

    # color mode
    def set_color_mode(self, c_mode):
        send_data.set_rpi_color_mode_val(c_mode)
    def get_color_mode(self):
        return send_data.get_rpi_color_mode_val()

    #color sensor value
    def get_ambient(self):
        if rev_data.get_spi_color_mode_val() == 1:
            return rev_data.get_ambient_val()
        else:
            return -1

    def get_color_code(self):
        if rev_data.get_spi_color_mode_val() == 2:
            return rev_data.get_color_val()
        else:
            return -1
    
    def get_reflection(self):
        if rev_data.get_spi_color_mode_val() == 3:
            return rev_data.get_reflect_val()
        else:
            return -1

    def get_rgb(self):
        if rev_data.get_spi_color_mode_val() == 4:
            rgb = [rev_data.get_r_val(), rev_data.get_g_val(), rev_data.get_b_val()]
            return rgb
        else:
            rgb = [-1, -1, -1]
            return rgb

    def get_rgb_r(self):
        if rev_data.get_spi_color_mode_val() == 4:
            return rev_data.get_r_val()
        else:
            return -1
    
    def get_rgb_g(self):
        if rev_data.get_spi_color_mode_val() == 4:
            return rev_data.get_g_val()
        else:
            return -1
    
    def get_rgb_b(self):
        if rev_data.get_spi_color_mode_val() == 4:
            return rev_data.get_b_val()
        else:
            return -1
    
color_sensor = colorSensorClass()

class ultrasonicSensorClass:
    def __init__(self):
        pass

    # ultrasonic sensor mode
    def set_ultrasonic_mode(self,u_mode):
        send_data.set_rpi_ultrasonic_mode_val(u_mode)
    def get_ultrasonic_mode(self):
        return send_data.get_rpi_ultrasonic_mode_val()

    # get distance
    def get_distance(self):
        if rev_data.get_spi_ultrasonic_mode_val() == 1:
            return rev_data.get_distance_val()
        else:
            return -1
    
    # get presence
    def get_presence(self):
        if rev_data.get_spi_ultrasonic_mode_val() == 2:
            return rev_data.get_presence_val()
        else:
            return -1

ultrasonic_sensor = ultrasonicSensorClass()

class imuClass:
    def __init__(self):
        pass

    def init(self):
        send_data.set_imu_init_val(True)

    def get_x_angular_velocity(self):
        return rev_data.get_x_ang_vel_val()

imu = imuClass()

class buttonClass:
    def __init__(self):
        pass

    def get_pressed_button_command(self):
        return rev_data.get_button_status_val()
    
    # is button pressed
    def is_center_pressed(self):
        if rev_data.get_button_status_val() & 0b10000:
            return True
        else:
            return False

    def is_left_pressed(self):
        if rev_data.get_button_status_val() & 0b1:
            return True
        else:
            return False

    def is_right_pressed(self):
        if rev_data.get_button_status_val() & 0b10:
            return True
        else:
            return False

    def is_bluetooth_pressed(self):
        if rev_data.get_touch_sensor_status_val() == 2048:
            return True
        else:
            return False

button = buttonClass()


class hubPowerClass:
    def __init__(self):
        pass

    def get_hub_voltage(self):
        return rev_data.get_voltage_val()

    def get_hub_current(self):
        return rev_data.get_current_val()

power = hubPowerClass()