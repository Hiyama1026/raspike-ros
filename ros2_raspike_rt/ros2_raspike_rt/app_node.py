import rclpy
from rclpy.node import Node

from .tools.spike_val import *
from .tools.api import *

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
        self.timer = self.create_timer(1, self.app_timer)
        self.test_seq = 0
        self.is_start = False
        self.reflection_val = 0
        self.diff = [0, 0]
        self.pre_i_val = 0
        self.steering_amount = 0

        # ログ出力
        self.get_logger().info('app init done')

    '''
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
        send_data.set_left_speed(int(bace_speed + (self.steering_amount * left_edge)))
        send_data.set_right_speed(int(bace_speed - (self.steering_amount * left_edge)))

    # timer callback
    def app_timer(self):
        if rev_data.get_button_status() == 16:
            self.is_start = True
        if self.is_start == False:
            return

        self.reflection_val = rev_data.get_reflect()

        self.steering_amount_calculation()
        self.motor_drive_control()
    '''
    
    # テスト
    def app_timer(self):
        #print('test')

        print(color.get_color_code())
        print(color.get_reflection())
        print('---')
        #motor.set_wheel_speed(self.test_seq, -self.test_seq)
    
        #print('     -r-')
        #print(rev_data.get_right_count())
        #print('     -l-')
        #print(rev_data.get_left_count())

        self.test_seq += 10
    
