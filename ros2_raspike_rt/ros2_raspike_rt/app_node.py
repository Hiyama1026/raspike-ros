import rclpy
from rclpy.node import Node
import time

from ros2_raspike_rt.lib.rpi_ros2_node import *
from ros2_raspike_rt.lib.spike_val import *
from ros2_raspike_rt.lib.api import *

white_brightness = 200
brack_brightness = 10
left_edge = -1
right_edge = 1

kp = 0.2
kd = 0.05
ki = 0.01
bace_speed = 40
sta_limit = 25

'''
ライントレース サンプルプログラム
'''

class appNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("app_node")
        # タイマーの生成
        self.timer = self.create_timer(0.01, self.app_timer)    # 引数：タイマーコールバック呼び出し周期，タイマーコールバック関数
        self.is_start = False
        self.rgb_val = [0, 0, 0]
        self.diff = [0, 0]
        self.pre_i_val = 0
        self.steering_amount = 0
        self.is_first = True

        # ログ出力
        self.get_logger().info('app init done')

    # ライントレース
    def steering_amount_calculation(self):
        target_brightness = (white_brightness + brack_brightness) / 2

        diff_brightness = target_brightness - self.rgb_val[2]
        delta_t = 10

        self.diff[1] = self.diff[0]
        self.diff[0] = int(diff_brightness)

        p_val = diff_brightness
        i_val = self.pre_i_val + (self.diff[0] + self.diff[1]) * delta_t / 2
        d_val = int((self.diff[0] + self.diff[1]) / 1)

        self.steering_amount = (kp * p_val) + (kd * d_val) + (ki * i_val)

        if self.steering_amount >= sta_limit:
            self.steering_amount = sta_limit
        elif self.steering_amount <= -sta_limit:
            self.steering_amount = -sta_limit

    def motor_drive_control(self):
        motor.set_left_motor_speed(int(bace_speed + (self.steering_amount * left_edge)))
        motor.set_right_motor_speed(int(bace_speed - (self.steering_amount * left_edge)))


    # timer callback
    def app_timer(self):
        if button.is_center_pressed():
            self.is_start = True
        if self.is_start == False:
            color_sensor.set_color_mode(4)
            return
        if self.is_first:
            speaker.play_tone(8, 200)
            time.sleep(1)
            self.is_first = False

        color_sensor.set_color_mode(4)
        self.rgb_val = color_sensor.get_rgb()

        self.steering_amount_calculation()
        self.motor_drive_control()


# メイン
def main(args=None):
    # ros2 start
    rpi_ros2_node_start(args)

if __name__ == "__main__":
    main()