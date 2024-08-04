#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/int8.hpp"
#include "raspike_uros_msg/msg/motor_speed_message.hpp"
#include "raspike_uros_msg/msg/spike_dev_status_message.hpp"
#include "raspike_uros_msg/msg/button_status_message.hpp"
#include "raspike_uros_msg/msg/speaker_message.hpp"
#include "wall_stop_cpp/wall_stop_controller.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int current_ultrasonic_mode;
int current_distance;
bool stop_flag = true;
int current_but_status = 0;
int center_but_status = 0;
int pre_center_button = 0;

class CppWallStopNode : public rclcpp::Node
{
public:
  CppWallStopNode() : Node("cpp_wall_stop_node"), count_(0)
  {
    // QoS設定を作成
    rclcpp::QoS qos_settings = rclcpp::QoS(10).best_effort();

    // パブリッシャーを作成
    speed_publisher_ = this->create_publisher<raspike_uros_msg::msg::MotorSpeedMessage>("wheel_motor_speeds", qos_settings);
    umode_publisher_ = this->create_publisher<std_msgs::msg::Int8>("ultrasonic_sensor_mode", 10);
    speaker_publisher_ = this->create_publisher<raspike_uros_msg::msg::SpeakerMessage>("speaker_tone", 10);
    // サブスクライバーを作成
    hub_status_subscriber_ = this->create_subscription<raspike_uros_msg::msg::SpikeDevStatusMessage>(
      "spike_device_status", qos_settings, std::bind(&CppWallStopNode::hub_status_callback, this, _1));
    button_status_subscriber_ = this->create_subscription<raspike_uros_msg::msg::ButtonStatusMessage>(
      "spike_button_status", qos_settings, std::bind(&CppWallStopNode::button_status_callback, this, _1));
    // タイマーを作成
    timer_ = this->create_wall_timer(
      10ms, std::bind(&CppWallStopNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // 距離センサモードをRGBモードに変更
    if (current_ultrasonic_mode != 1) {
      auto umode_msg = std_msgs::msg::Int8();
      umode_msg.data = 1;     // 1:distance (ref: ROS2_GUIDE.md)
      umode_publisher_->publish(umode_msg);
      return;
    }

    // センターボタンが押されたらスピーカを鳴らしてstop_flagを更新
    if (center_but_status == BT_CENTER && pre_center_button != BT_CENTER) {
      RCLCPP_INFO(this->get_logger(), "CENTER pressed.");
      
      auto speaker_msg = raspike_uros_msg::msg::SpeakerMessage();
      speaker_msg.tone = 4;       // F4 (ref: ROS2_GUIDE.md)
      speaker_msg.duration = 500;
      speaker_publisher_->publish(speaker_msg);

      stop_flag = !stop_flag;
    }
    pre_center_button = center_but_status;

    auto mspeed_msg = raspike_uros_msg::msg::MotorSpeedMessage();

    // stop_flagに応じてモータを停止・再開
    if (stop_flag) {
      mspeed_msg.right_motor_speed = 0; 
      mspeed_msg.left_motor_speed = 0; 
      mspeed_msg.arm_motor_speed = 0;
      speed_publisher_->publish(mspeed_msg);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "distance %d", current_distance);

    // 物体との距離がBOUNDARY_DISTANCEを下回ったら停止
    if (current_distance < BOUNDARY_DISTANCE && current_distance != -1) {
      mspeed_msg.right_motor_speed = 0; 
      mspeed_msg.left_motor_speed = 0; 
      mspeed_msg.arm_motor_speed = 0; 
    }
    else {
      mspeed_msg.right_motor_speed = WHEEL_SPEED; 
      mspeed_msg.left_motor_speed = WHEEL_SPEED; 
      mspeed_msg.arm_motor_speed = 0; 
    }
    speed_publisher_->publish(mspeed_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<raspike_uros_msg::msg::MotorSpeedMessage>::SharedPtr speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr umode_publisher_;
  rclcpp::Publisher<raspike_uros_msg::msg::SpeakerMessage>::SharedPtr speaker_publisher_;
  size_t count_;

private:
  void hub_status_callback(const raspike_uros_msg::msg::SpikeDevStatusMessage::SharedPtr msg) const
  {
    current_ultrasonic_mode = msg->ultrasonic_mode_id;
    current_distance = msg->ultrasonic_sensor;
  }
  rclcpp::Subscription<raspike_uros_msg::msg::SpikeDevStatusMessage>::SharedPtr hub_status_subscriber_;

private:
  void button_status_callback(const raspike_uros_msg::msg::ButtonStatusMessage::SharedPtr msg) const
  {
    current_but_status = msg->button;
    // センターボタンの押下状態を取得
    center_but_status = current_but_status & 0b00010000;
  }
  rclcpp::Subscription<raspike_uros_msg::msg::ButtonStatusMessage>::SharedPtr button_status_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppWallStopNode>());
  rclcpp::shutdown();
  return 0;
}