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

from concurrent.futures import ThreadPoolExecutor 
from rclpy.executors import MultiThreadedExecutor

from .app_node import *
from .lib.spike_val import *


# サブスクライバーノード
class rasberryPiNode(Node):
    # 初期化
    def __init__(self):
        super().__init__("rpi_ros2_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        # 送受信
        self.rev_color_mode = 0
        self.rev_ultrasonic_mode = 0
        self.pre_ultrasonic_mode = 0
        self.send_ultrasonic_mode = 0

        # パブリッシャーの生成
        self.motor_speed_publisher = self.create_publisher(MotorSpeedMessage, "wheel_motor_speeds", qos_profile)
        self.motor_reset_count_publisher = self.create_publisher(MotorResetMessage, "motor_reset_count", 10)
        self.color_mode_publisher = self.create_publisher(Int8, "color_sensor_mode", qos_profile)
        self.ultrasonic_mode_publisher = self.create_publisher(Int8, "ultrasonic_sensor_mode", 10)
        self.imu_init_publisher = self.create_publisher(Bool, "imu_init", 10)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.01, self.timer_on_tick)                           #wheel_speed
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikeDevStatusMessage, "spike_device_status", self.dev_status_on_subscribe, qos_profile)
        self.button_status_subscription = self.create_subscription(
            ButtonStatusMessage, "spike_button_status", self.button_status_on_subscribe, qos_profile)
        self.hub_status_subscription = self.create_subscription(
            SpikePowerStatusMessage, "spike_power_status", self.hub_status_on_subscribe, qos_profile)
        
        self.get_logger().info("line tracer node init")

    def timer_on_tick(self):
        # メッセージの生成
        motor_speed = MotorSpeedMessage()
        reset_count = MotorResetMessage()
        color_mode = Int8()
        ultrasonic_mode = Int8()   
        imu_init = Bool()        

        motor_speed.right_motor_speed = send_data.get_right_speed_val()
        motor_speed.left_motor_speed = send_data.get_left_speed_val()
        motor_speed.arm_motor_speed = send_data.get_arm_speed_val()
        motor_speed.right_motor_stop_brake = send_data.get_right_stop_brake_val()
        motor_speed.left_motor_stop_brake = send_data.get_left_stop_brake_val()
        motor_speed.arm_motor_stop_brake = send_data.get_arm_stop_brake_val()
        # motor speed パブリッシュ
        self.motor_speed_publisher.publish(motor_speed)

        #color_mode.data = 4     #固定
        color_mode.data = send_data.get_rpi_color_mode_val()
        # color mode パブリッシュ
        self.color_mode_publisher.publish(color_mode)

        #motor reset count
        reset_count.right_motor_reset = send_data.get_right_reset_val()
        reset_count.left_motor_reset = send_data.get_left_reset_val()
        reset_count.arm_motor_reset = send_data.get_arm_reset_val()
        if (reset_count.right_motor_reset or 
           reset_count.left_motor_reset  or 
           reset_count.arm_motor_reset):
            # reset count パブリッシュ 
            self.motor_reset_count_publisher.publish(reset_count)
            send_data.set_right_reset_val(False)
            send_data.set_left_reset_val(False)
            send_data.set_arm_reset_val(False)
        
        # ultrasonic mode
        self.pre_ultrasonic_mode = self.send_ultrasonic_mode
        self.send_ultrasonic_mode = send_data.get_rpi_ultrasonic_mode_val()
        if self.pre_ultrasonic_mode != self.send_ultrasonic_mode:
            ultrasonic_mode.data = self.send_ultrasonic_mode
            self.ultrasonic_mode_publisher.publish(ultrasonic_mode)

        # imu init
        imu_init.data = send_data.get_imu_init_val()
        if imu_init.data:
            self.imu_init_publisher.publish(imu_init)
            send_data.set_imu_init_val(False)
        

    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):

        self.rev_color_mode = devise_status.color_mode_id
        rev_data.set_spi_color_mode_val(self.rev_color_mode)
        if self.rev_color_mode == 1:
            rev_data.set_ambient_val(devise_status.color_sensor_value_1)
        elif self.rev_color_mode == 2:
            rev_data.set_color_val(devise_status.color_sensor_value_1)
        elif self.rev_color_mode == 3:
            rev_data.set_reflect_val(devise_status.color_sensor_value_1)
        elif self.rev_color_mode == 4:
            rev_data.set_r_val(devise_status.color_sensor_value_1)
            rev_data.set_g_val(devise_status.color_sensor_value_2)
            rev_data.set_b_val(devise_status.color_sensor_value_3)

        self.rev_ultrasonic_mode = devise_status.ultrasonic_mode_id
        rev_data.set_spi_ultrasonic_mode_val(self.rev_ultrasonic_mode)
        if self.rev_ultrasonic_mode == 1:
            rev_data.set_distance_val(devise_status.ultrasonic_sensor)
        elif self.rev_ultrasonic_mode == 2:
            rev_data.set_presence_val(devise_status.ultrasonic_sensor)

        rev_data.set_arm_count_val(devise_status.arm_count)
        rev_data.set_right_count_val(devise_status.right_count)
        rev_data.set_left_count_val(devise_status.left_count)

        rev_data.set_x_ang_vel_val(devise_status.gyro_sensor)
        
    def button_status_on_subscribe(self, button_status):
        #self.get_logger().info("touch_sensor : " + str(button_status.touch_sensor))
        #self.get_logger().info("button : " + str(button_status.button))

        rev_data.set_button_status_val(button_status.button)
        rev_data.set_touch_sensor_status_val(button_status.touch_sensor)

    def hub_status_on_subscribe(self, hub_status):
        rev_data.set_voltage_val(hub_status.voltage)
        rev_data.set_current_val(hub_status.current)


# メイン
def main(args=None):
    # ROS通信の初期化
    rclpy.init(args=args)
    exec = MultiThreadedExecutor()

    # ノードの生成
    node = rasberryPiNode()
    app_node = appNode()

    # ノード終了まで待機
    exec.add_node(node)
    exec.add_node(app_node)
    exec.spin()

    # ノードの破棄
    exec.shutdown()
    node.destroy_node()
    app_node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()


if __name__ == "__main__":
    main()
