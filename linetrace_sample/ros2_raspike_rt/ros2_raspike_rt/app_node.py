import rclpy
from rclpy.node import Node

from .spike_val import *

class appNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("app_node")
        # タイマーの生成
        self.timer = self.create_timer(0.1, self.app_timer)
        self.test_seq = 0

        # ログ出力
        self.get_logger().info('app init done')

    # timer callback
    def app_timer(self):
        #print('test')

        print(rev_data.get_reflection())

        #print('     -r-')
        #print(rev_data.get_right_count())
        #print('     -l-')
        #print(rev_data.get_left_count())

        self.test_seq += 1

