from .spike_val import *

class motorClass:
    def __init__(self):
        pass

    def set_wheel_speed(self, left_speed, right_speed):
        send_data.set_left_speed(left_speed)
        send_data.set_right_speed(right_speed)

    def set_arm_speed(self, arm_speed):
        send_data.set_arm_speed(arm_speed)


motor = motorClass()
    
class colorSensorClass:
    def __init__(self):
        pass

    def get_color_code(self):
        if rev_data.get_spi_color_mode() == 2:
            return rev_data.get_color()
        else:
            while rev_data.get_spi_color_mode() != 2:
                send_data.set_rpi_color_mode(2)
            return rev_data.get_color()
    
    def get_reflection(self):
        if rev_data.get_spi_color_mode() == 3:
            return rev_data.get_reflect()
        else:
            while rev_data.get_spi_color_mode() != 3:
                send_data.set_rpi_color_mode(3)
            return rev_data.get_reflect()

color = colorSensorClass()