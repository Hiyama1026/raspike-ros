import rclpy
from rclpy.node import Node

from .lib.spike_val import *
from .lib.api import *

white_brightness = 74
brack_brightness = 4
left_edge = -1
right_edge = 1

kp = 0.59
kd = 0.04
ki = 0.0
bace_speed = 40

class appNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("app_node")
        # タイマーの生成
        self.timer = self.create_timer(0.01, self.app_timer)    # 引数：タイマーコールバック呼び出し周期，タイマーコールバック関数
        self.is_start = False
        self.reflection_val = 0
        self.diff = [0, 0]
        self.pre_i_val = 0
        self.steering_amount = 0

        # ログ出力
        self.get_logger().info('app init done')


    # センサ値取得
    def reliable_get_reflection(self):
        get_ref_value = color_sensor.get_reflection()
        while get_ref_value < 0:
            get_ref_value = color_sensor.get_reflection()
        
        return get_ref_value

    
    # ライントレース
    def steering_amount_calculation(self):
        target_brightness = (white_brightness - brack_brightness) / 2
        
        diff_brightness = target_brightness - self.reflection_val
        delta_t = 10

        self.diff[1] = self.diff[0]
        self.diff[0] = int(diff_brightness)

        p_val = diff_brightness
        i_val = self.pre_i_val + (self.diff[0] + self.diff[1]) * delta_t / 2
        d_val = int((self.diff[0] - self.diff[1]) / 1)

        self.steering_amount = (kp * p_val) + (kd * d_val) + (ki * i_val)

    def motor_drive_control(self):
        send_data.set_left_speed_val(int(bace_speed + (self.steering_amount * left_edge)))
        send_data.set_right_speed_val(int(bace_speed - (self.steering_amount * left_edge)))

    # timer callback
    def app_timer(self):
        if button.is_center_pressed():
            self.is_start = True
        if self.is_start == False:
            return

        color_sensor.set_color_mode(3)
        self.reflection_val = self.reliable_get_reflection()

        self.steering_amount_calculation()
        self.motor_drive_control()
