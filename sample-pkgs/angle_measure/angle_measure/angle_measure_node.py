import rclpy
import time
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from raspike_uros_msg.msg import MotorSpeedMessage        #motor speed
from raspike_uros_msg.msg import MotorResetMessage             #reset count
from raspike_uros_msg.msg import SpikeDevStatusMessage
from raspike_uros_msg.msg import ButtonStatusMessage
from raspike_uros_msg.msg import SpikePowerStatusMessage
from rclpy.node import Node
from rclpy.qos import QoSProfile


# サブスクライバーノード
class AngleMeasureNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("angle_measure_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        self.is_start = False
        self.steering_amount = 0
        # 受信メッセージ
        self.rev_y_angle = 0
        self.pre_rev_y_angle = 0
        self.pre_rev_z_angle = 0
        self.rev_z_angle = 0
        self.pre_button_val = 0
        self.rev_button_val = 0

        # パブリッシャーの生成
        self.imu_init_publisher = self.create_publisher(Bool, "imu_init", 10)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.01, self.timer_on_tick)
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikeDevStatusMessage, "spike_device_status", self.dev_status_on_subscribe, qos_profile)
        self.button_status_subscription = self.create_subscription(
            ButtonStatusMessage, "spike_button_status", self.button_status_on_subscribe, qos_profile)
        
        self.get_logger().info("angle_measure_node init")

    def timer_on_tick(self):
        # 角度が変化したら表示
        if self.rev_y_angle != self.pre_rev_y_angle or self.rev_z_angle != self.pre_rev_z_angle:
            self.get_logger().info("---")
            self.get_logger().info("y_angle : " + str(self.rev_y_angle))
            self.get_logger().info("z_angle : " + str(self.rev_z_angle))
        self.pre_rev_y_angle = self.rev_y_angle
        self.pre_rev_z_angle = self.rev_z_angle
        
        #センターボタンが押されたら角度をリセット
        if (self.rev_button_val & 0b00010000) and not (self.pre_button_val & 0b00010000):
            imu_init_msg = Bool()
            imu_init_msg.data = True
            self.imu_init_publisher.publish(imu_init_msg)
            self.get_logger().info("")
            self.get_logger().info("reset angle")
            self.get_logger().info("")
        self.pre_button_val = self.rev_button_val

    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):
        self.rev_y_angle = devise_status.y_angle
        self.rev_z_angle = devise_status.z_angle
        
    def button_status_on_subscribe(self, button_status):
        self.rev_button_val = button_status.button

# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)

    # ノードの生成
    node = AngleMeasureNode()

    # ノード終了まで待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()


if __name__ == "__main__":
    main()
