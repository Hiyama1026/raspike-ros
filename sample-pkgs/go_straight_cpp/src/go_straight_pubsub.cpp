#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/int8.hpp"
#include "raspike_uros_msg/msg/motor_speed_message.hpp"
#include "raspike_uros_msg/msg/spike_dev_status_message.hpp"
#include "raspike_uros_msg/msg/button_status_message.hpp"
#include "raspike_uros_msg/msg/speaker_message.hpp"
#include "go_straight_cpp/go_straight_pubsub.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

int current_color_mode;
int current_rgb_b;
bool stop_flag = true;
int current_but_status = 0;
int center_but_status = 0;
int pre_center_button = 0;

class CppGoStraightPubSub : public rclcpp::Node
{
public:
  CppGoStraightPubSub() : Node("cpp_go_straight_pubsub"), count_(0)
  {
    // QoS設定を作成
    rclcpp::QoS qos_settings = rclcpp::QoS(10).best_effort();

    // パブリッシャーを作成
    speed_publisher_ = this->create_publisher<raspike_uros_msg::msg::MotorSpeedMessage>("wheel_motor_speeds", qos_settings);
    cmode_publisher_ = this->create_publisher<std_msgs::msg::Int8>("color_sensor_mode", 10);
    speaker_publisher_ = this->create_publisher<raspike_uros_msg::msg::SpeakerMessage>("speaker_tone", 10);
    // サブスクライバーを作成
    hub_status_subscriber_ = this->create_subscription<raspike_uros_msg::msg::SpikeDevStatusMessage>(
      "spike_device_status", qos_settings, std::bind(&CppGoStraightPubSub::hub_status_callback, this, _1));
    button_status_subscriber_ = this->create_subscription<raspike_uros_msg::msg::ButtonStatusMessage>(
      "spike_button_status", qos_settings, std::bind(&CppGoStraightPubSub::button_status_callback, this, _1));
    // タイマーを作成
    timer_ = this->create_wall_timer(
      10ms, std::bind(&CppGoStraightPubSub::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // カラーセンサモードをRGBモードに変更
    if (current_color_mode != 4) {
      auto cmode_msg = std_msgs::msg::Int8();
      cmode_msg.data = 4;     // 1:ambient, 2:color, 3:reflection, 4:RGB (ref: ROS2_GUIDE.md)
      cmode_publisher_->publish(cmode_msg);
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
    // 黒線を見つけたら停止
    if (current_rgb_b < BOUNDARY_VALUE) {
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
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmode_publisher_;
  rclcpp::Publisher<raspike_uros_msg::msg::SpeakerMessage>::SharedPtr speaker_publisher_;
  size_t count_;

private:
  void hub_status_callback(const raspike_uros_msg::msg::SpikeDevStatusMessage::SharedPtr msg) const
  {
    current_color_mode = msg->color_mode_id;
    current_rgb_b = msg->color_sensor_value_3;
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
  rclcpp::spin(std::make_shared<CppGoStraightPubSub>());
  rclcpp::shutdown();
  return 0;
}