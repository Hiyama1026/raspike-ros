class rev_data_class:
    def __init__(self):
        self.__arm_count = 0
        self.__right_count = 0
        self.__left_count = 0 
        self.__spike_color_mode = 0
        self.__color_sensor_ambient = 0
        self.__color_sensor_color_code = 0
        self.__color_sensor_reflection = 0
        self.__color_sensor_r = 0
        self.__color_sensor_g = 0
        self.__color_sensor_b = 0
        self.__spike_ultrasonic_mode = 0
        self.__ultrasonic_distance = 0
        self.__ultrasonic_presence = 0
        self.__gyro_x_angular_velocity = 0
        self.__hub_button_status = 0
        self.__hub_touch_sensor_status = 0
        self.__hub_voltage = 0
        self.__hub_current = 0

    # arm motor count
    def set_arm_count(self, a_count):   
        self.__arm_count = a_count
    
    def get_arm_count(self):          
        return self.__arm_count
    
    # right motor count
    def set_right_count(self, r_count):   
        self.__right_count = r_count
    
    def get_right_count(self):          
        return self.__right_count
    
    # left motor count
    def set_left_count(self, l_count):   
        self.__left_count = l_count
    
    def get_left_count(self):          
        return self.__left_count

    # spike color mode
    def set_spi_color_mode(self, s_col_mode):   
        self.__spike_color_mode = s_col_mode
    
    def get_spi_color_mode(self):          
        return self.__spike_color_mode
    
    # colr sensor value
    def set_ambient(self, col_ambient):   
        self.__color_sensor_ambient = col_ambient
    
    def get_ambient(self):          
        return self.__color_sensor_ambient

    def set_color(self, col_color):   
        self.__color_sensor_color_code = col_color
    
    def get_color(self):          
        return self.__color_sensor_color_code
    
    def set_reflection(self, col_reflection):   
        self.__color_sensor_reflection = col_reflection
    
    def get_reflection(self):          
        return self.__color_sensor_reflection

    def set_r(self, col_r):   
        self.__color_sensor_r = col_r
    
    def get_r(self):          
        return self.__color_sensor_r

    def set_g(self, col_g):   
        self.__color_sensor_g = col_g
    
    def get_g(self):          
        return self.__color_sensor_g

    def set_b(self, col_b):   
        self.__color_sensor_b = col_b
    
    def get_b(self):          
        return self.__color_sensor_b
    
    # spike ultrasonic mode
    def set_spi_ultrasonic_mode(self, s_ult_mode):   
        self.__spike_ultrasonic_mode = s_ult_mode
    
    def get_spi_ultrasonic_mode(self):          
        return self.__spike_ultrasonic_mode

    # ultrasonic sensor value
    def set_distance(self, ult_distance):   
        self.__ultrasonic_distance = ult_distance
    
    def get_spi_ultrasonic_mode(self):          
        return self.__ultrasonic_distance

    def set_presence(self, ult_presence):   
        self.__ultrasonic_presence = ult_presence
    
    def get_spi_ultrasonic_mode(self):          
        return self.__ultrasonic_presence

    # gyro sensor
    def set_x_ang_vel(self, gyro_val):   
        self.__gyro_x_angular_velocity = gyro_val
    
    def get_x_ang_vel(self):          
        return self.__gyro_x_angular_velocity

    # button status
    def set_button_status(self, button):   
        self.__hub_button_status = button
    
    def get_button_status(self):          
        return self.__hub_button_status

    # touch sensor status
    def set_touch_sensor_status(self, touch_sensor):   
        self.__hub_touch_sensor_status = touch_sensor
    
    def get_touch_sensor_status(self):          
        return self.__hub_touch_sensor_status

    # hub power status
    def set_voltage(self, volt):   
        self.__hub_voltage = volt
    
    def get_voltage(self):          
        return self.__hub_voltage

    def set_current(self, current):   
        self.__hub_current = current
    
    def get_current(self):          
        return self.__hub_current


rev_data = rev_data_class()


class send_data_class:
    def __init__(self):
        self.__arm_motor_speed = 0 
        self.__right_motor_speed = 0 
        self.__left_motor_speed = 0 
        self.__arm_motor_stop_brake = 0
        self.__right_motor_stop_brake = 0
        self.__left_motor_stop_brake = 0
        self.__arm_motor_reset = False 
        self.__right_motor_reset = False
        self.__left_motor_reset = False
        self.__raspi_color_sensor_mode = 0

    # motor speed
    def set_arm_speed(self, a_speed):
        self.__arm_motor_speed = a_speed

    def get_arm_speed(self): 
        return self.__arm_motor_speed
    
    def set_right_speed(self, r_speed):
        self.__right_motor_speed = r_speed

    def get_right_speed(self):
        return self.__right_motor_speed

    def set_left_speed(self, l_speed):
        self.__left_motor_speed = l_speed

    def get_left_speed(self):
        return self.__left_motor_speed

    # motor stop or brake
    def set_arm_stop_brake(self, a_stop_brake):
        self.__arm_motor_stop_brake = a_stop_brake
    
    def get_arm_stop_brake(self): 
        return self.__arm_motor_stop_brake
    
    def set_right_stop_brake(self, r_stop_brake):
        self.__right_motor_stop_brake = r_stop_brake
    
    def get_right_stop_brake(self):
        return self.__right_motor_stop_brake

    def set_left_stop_brake(self, l_stop_brake):
        self.__left_motor_stop_brake = l_stop_brake
        
    def get_left_stop_brake(self):
        return self.__left_motor_stop_brake

    # motor reset count
    def set_arm_reset(self, a_reset):
        self.__arm_motor_reset = a_reset
    
    def get_arm_reset(self): 
        return self.__arm_motor_reset
    
    def set_right_reset(self, r_reset):
        self.__right_motor_reset = r_reset
    
    def get_right_reset(self):
        return self.__right_motor_reset

    def set_left_reset(self, l_reset):
        self.__left_motor_reset = l_reset
        
    def get_left_reset(self):
        return self.__left_motor_reset
    
    # color mode
    def set_rpi_color_mode(self, r_col_mode):
        self.__raspi_color_sensor_mode = r_col_mode
        
    def get_rpi_color_mode(self):
        return self.__raspi_color_sensor_mode

send_data = send_data_class()

'''
ToDo

rgb挙動確認
距離センサー挙動確認
imu init
'''