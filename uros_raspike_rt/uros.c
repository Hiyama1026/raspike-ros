#include <kernel.h>
#include <t_syslog.h>
#include <micro_ros_asp.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>

#include <sensor_msgs/msg/range.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

/*spike hub&pup lib*/
#include <spike/hub/display.h>
#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/ultrasonicsensor.h"
#include "spike/hub/button.h"
#include "spike/hub/speaker.h"
#include <spike/hub/light.h>
#include "pbio/color.h"
#include <spike/hub/imu.h>
#include <spike/hub/battery.h>
#include <pbdrv/battery.h>

#include <stdio.h>
#include <time.h>
#include "uros.h"

//u_ros custom message
#include <std_msgs/msg/int8.h>		//int8
#include <std_msgs/msg/bool.h>		//bool
#include <raspike_uros_msg/msg/button_status_message.h>			//button status
#include <raspike_uros_msg/msg/spike_dev_status_message.h>		//device status
#include <raspike_uros_msg/msg/spike_power_status_message.h>	//power status
#include <raspike_uros_msg/msg/motor_speed_message.h>			//motor speed
#include <raspike_uros_msg/msg/motor_reset_message.h>			//reset count
#include <raspike_uros_msg/msg/speaker_message.h>			//speaker


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


static pup_color_hsv_t raspike_rt_detectable_color[] = {
    { PBIO_COLOR_HUE_RED, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_YELLOW, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_GREEN, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_BLUE, 0b01100100, 0b01100100 }, 
    {0, 0, 0b01100100},     //WHITE
    {0, 0, 0b1010},         //BRACK
    {0, 0, 0} ,             //NON
};

#if 0
static pup_color_hsv_t detectable_color_for_EV3[] = {
    { PBIO_COLOR_HUE_RED, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_YELLOW, 0b01100100, 0b01100100 },
    { PBIO_COLOR_HUE_GREEN, 0b01100100, 0b01100100 }, 
    { PBIO_COLOR_HUE_BLUE, 0b01100100, 0b01100100 }, 
    {0b00101000, 0b01100100, 0b00100011},       //BROWN (H=40, S=100, V=35)
    {0, 0, 0b01100100},                         //WHITE
    {0, 0, 0b1010},                             //BRACK
    {0, 0, 0} ,                                 //NON
};
#endif

rcl_publisher_t dev_status_publisher;
rcl_publisher_t button_status_publisher;
rcl_publisher_t power_status_publisher;
rcl_subscription_t motor_speed_subscriber;
rcl_subscription_t reset_count_subscriber;
rcl_subscription_t color_mode_subscriber;
rcl_subscription_t ultrasonic_mode_subscriber;
rcl_subscription_t imu_init_subscriber;
rcl_subscription_t speaker_subscriber;

raspike_uros_msg__msg__SpikeDevStatusMessage device_status;
raspike_uros_msg__msg__ButtonStatusMessage hub_button_msg;
raspike_uros_msg__msg__SpikePowerStatusMessage power_msg;
raspike_uros_msg__msg__MotorSpeedMessage motor_speed;
raspike_uros_msg__msg__MotorResetMessage reset_count;
raspike_uros_msg__msg__SpeakerMessage speaker_tone_val;
std_msgs__msg__Int8 color_sensor_mode;
std_msgs__msg__Int8 ultrasonic_sensor_mode;
//std_msgs__msg__Int8 speaker_tone_val;
std_msgs__msg__Bool imu_init;

static pbio_error_t r_err;     
static pbio_error_t l_err;      
static pbio_error_t a_err;
static pup_motor_t *a_motor;       //arm motor
static pup_motor_t *r_motor;       //right motor
static pup_motor_t *l_motor;       //left motor
static pup_device_t *col;          //color sensor
static pup_device_t *ult;          //ultrasonic sensor
int16_t send_color_value_1;
int16_t send_color_value_2;
int16_t send_color_value_3;
int8_t send_color_mode_id;
int8_t send_ultrasonic_mode_id;
int16_t send_ultrasonic_value;
int8_t current_color_mode;
int8_t current_ultrasonic_mode;
int8_t temp_speaker_frequency;
int16_t speaker_frequency;
static timer_count = 0;
bool speaker_enabled = false;
int16_t speaker_play_duration = 0;
int16_t speaker_cnt = 0;

int button_state;
int touch_sensor_state;
int pre_button_state;
int pre_touch_sensor_state;

SYSTIM *p_systim;


void error_loop(rcl_ret_t temp_rc){
	syslog(LOG_NOTICE, "error_loop %d\n", temp_rc);

    hub_display_image(img_sad);             //ディスプレイ表示
    hub_light_on_color(PBIO_COLOR_RED);     //ステータスライトを赤く点灯する

	while(1){
		dly_tsk(100);
	}
}

void get_color_code(void){
    pup_color_hsv_t tmp_color_val;

    tmp_color_val = pup_color_sensor_color(col, true);
#if 0
    // for EV3
    switch (tmp_color_val.h){
        case 0:
            if (tmp_color_val.s == 100){            // RED (PBIO_COLOR_HUE_RED=0)
            send_color_value_1 = 5;
            }
            else if (tmp_color_val.v == 100){       // WHITE
                send_color_value_1 = 6;
            }
            else if (tmp_color_val.v == 10){        // BRACK
                send_color_value_1 = 1;
            }
            else{
                send_color_value_1 = -1;            // NONE
            }
            break;
        case PBIO_COLOR_HUE_BLUE:
            send_color_value_1 = 2;
            break;
        case PBIO_COLOR_HUE_GREEN:
            send_color_value_1 = 3;
            break;
        case PBIO_COLOR_HUE_YELLOW:
            send_color_value_1 = 4;
            break;
        case 40:                                    // BROWN
            send_color_value_1 = 7;
            break;
        default:
            send_color_value_1 = -1;                //NONE
    }
#endif

#if 1      
    switch(tmp_color_val.h){
        case 0:            
            if(tmp_color_val.s == 100){             //RED (PBIO_COLOR_HUE_RED=0)
                send_color_value_1 = 1;
            }
            else if(tmp_color_val.v == 100){        //WHITE
                send_color_value_1 = 5;
            }
            else if(tmp_color_val.v == 10){         //BRACK
                send_color_value_1 = 6;
            }
            else{
                send_color_value_1 = 0;            //NONE
            }
            break;
        case PBIO_COLOR_HUE_YELLOW:
            send_color_value_1 = 2;
            break;
        case PBIO_COLOR_HUE_GREEN:
            send_color_value_1 = 3;
            break;
        case PBIO_COLOR_HUE_BLUE:
            send_color_value_1 = 4;
            break;
        default:
            send_color_value_1 = -2;                //err
    }
#endif
    send_color_value_2 = 0;
    send_color_value_3 = 0;

    return;
}

void get_color_sensor_value(int8_t color_mode){
	switch (color_mode){
		pup_color_rgb_t color_rgb;
    	pup_color_hsv_t color_hsv;
            case 1:
                send_color_value_1 = pup_color_sensor_ambient(col);
                send_color_value_2 = 0;
                send_color_value_3 = 0;
                send_color_mode_id = 1;
                break;
            case 2:
                get_color_code();
                send_color_mode_id = 2;
                break;
            case 3:
                send_color_value_1 = pup_color_sensor_reflection(col);
                send_color_value_2 = 0;
                send_color_value_3 = 0;
                send_color_mode_id = 3;
                break;
            case 4:
                 color_rgb = pup_color_sensor_rgb(col);
                 color_hsv = pup_color_sensor_hsv(col, true);
                if(color_hsv.h==0 && color_hsv.s==0 && color_hsv.v==0)
                {
                }
                else{
                    send_color_value_1 = (int16_t)(color_rgb.r / 4);  
                    send_color_value_2 = (int16_t)(color_rgb.g / 4);          
                    send_color_value_3 = (int16_t)(color_rgb.b / 4);
                    send_color_mode_id = 4;
                }                
                break;
            default:
                send_color_mode_id = 0;
                break;
        }

}

void get_ultrasonic_sensor_value(int8_t ultrasonic_mode){
	switch (ultrasonic_mode){
            case 1:
                send_ultrasonic_value = pup_ultrasonic_sensor_distance(ult);
                send_ultrasonic_value = send_ultrasonic_value/10;
                if(send_ultrasonic_value < 0)   send_ultrasonic_value = -1;
                send_ultrasonic_mode_id = 1;
                break;
            case 2:
                if(pup_ultrasonic_sensor_presence(ult))     send_ultrasonic_value = 1;
                else                                        send_ultrasonic_value = 0;
                send_ultrasonic_mode_id = 2;
                break;
            default:
                send_ultrasonic_mode_id = 0;
        }
}


static inline hub_button_t hub_buttons_pressed(hub_button_t button_candidates)
{
  hub_button_t pressed;
  hub_button_is_pressed(&pressed);
  return pressed & button_candidates;
}

static int wait_for_hub_buttons(hub_button_t button_candidates)
{
    hub_button_t pressed_button;
    int button_command = 0;

    pressed_button = hub_buttons_pressed(button_candidates);

    if(pressed_button & HUB_BUTTON_BT)      return 2048;
    if(pressed_button & HUB_BUTTON_LEFT)    button_command += 1;
    if(pressed_button & HUB_BUTTON_RIGHT)   button_command += 2;
    if(pressed_button & HUB_BUTTON_CENTER)  button_command += 16;

    return button_command;
}

static void play_speaker(int code)
{
    switch (code)
    {
        case 1:
            hub_speaker_play_tone(NOTE_C4, SOUND_MANUAL_STOP);
            break;
        case 2:
            hub_speaker_play_tone(NOTE_D4, SOUND_MANUAL_STOP);
            break;
        case 3:
            hub_speaker_play_tone(NOTE_E4, SOUND_MANUAL_STOP);
            break;
        case 4:
            hub_speaker_play_tone(NOTE_F4, SOUND_MANUAL_STOP);
            break;
        case 5:
            hub_speaker_play_tone(NOTE_G4, SOUND_MANUAL_STOP);
            break;
        case 6:
            hub_speaker_play_tone(NOTE_A4, SOUND_MANUAL_STOP);
            break;
        case 7:
            hub_speaker_play_tone(NOTE_B4, SOUND_MANUAL_STOP);
            break;
        case 8:
            hub_speaker_play_tone(NOTE_C5, SOUND_MANUAL_STOP);
            break;
        case 9:
            hub_speaker_play_tone(NOTE_D5, SOUND_MANUAL_STOP);
            break;   
        case 10:
            hub_speaker_play_tone(NOTE_E5, SOUND_MANUAL_STOP);
            break;       
        default:
            break;
    }
    return;
}

void 
timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	float hub_angular_velocity[3];

	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		/*color*/
		get_color_sensor_value(current_color_mode);
		device_status.color_mode_id = send_color_mode_id;
		device_status.color_sensor_value_1 = send_color_value_1;
		device_status.color_sensor_value_2 = send_color_value_2;
		device_status.color_sensor_value_3 = send_color_value_3;
		/*ultrasonic*/
		get_ultrasonic_sensor_value(current_ultrasonic_mode);
		device_status.ultrasonic_mode_id = send_ultrasonic_mode_id;
		device_status.ultrasonic_sensor = send_ultrasonic_value;
		/*motor*/
		device_status.arm_count = pup_motor_get_count(a_motor);
		device_status.right_count = pup_motor_get_count(r_motor);
		device_status.left_count = pup_motor_get_count(l_motor);
		/*imu*/
		hub_imu_get_angular_velocity(&hub_angular_velocity[0]);
		device_status.gyro_sensor = hub_angular_velocity[0];

		RCSOFTCHECK(rcl_publish(&dev_status_publisher, &device_status, NULL));


		/*button_status_publisher*/
		button_state = wait_for_hub_buttons(HUB_BUTTON_RIGHT|HUB_BUTTON_LEFT|HUB_BUTTON_CENTER);
        touch_sensor_state = wait_for_hub_buttons(HUB_BUTTON_BT);
		if(pre_button_state != button_state){
			hub_button_msg.button = button_state;
			RCCHECK(rcl_publish(&button_status_publisher, (const void*)&hub_button_msg, NULL));
		}
		pre_button_state = button_state;
		if(pre_touch_sensor_state != touch_sensor_state){
			hub_button_msg.touch_sensor = touch_sensor_state;
			RCCHECK(rcl_publish(&button_status_publisher, (const void*)&hub_button_msg, NULL));
		}
		pre_touch_sensor_state = touch_sensor_state;

		/*power_status_publisher*/
		timer_count++;
		if(timer_count == 10){		//100ms周期
			power_msg.voltage = hub_battery_get_voltage();
			power_msg.current = hub_battery_get_current();
			RCSOFTCHECK(rcl_publish(&power_status_publisher, &power_msg, NULL));
			timer_count = 0;
		}

        if (speaker_enabled){       //speaker停止処理
            if (speaker_play_duration == speaker_cnt) {
                hub_speaker_stop();
                speaker_enabled = false;
            }
            else {
                speaker_cnt++;
            }
        }

	}
}


void motor_speed_callback(const void * msgin)
{
    const raspike_uros_msg__msg__MotorSpeedMessage * motor_speed = (const raspike_uros_msg__msg__MotorSpeedMessage *)msgin;

	if(motor_speed->arm_motor_stop_brake != 0){
		if(motor_speed->arm_motor_stop_brake)	pup_motor_stop(a_motor);		//arm_motor_stop_brake=1 -> stop
		else									pup_motor_brake(a_motor);		//arm_motor_stop_brake=2 -> brake
	}
	else{
        pup_motor_set_power(a_motor, motor_speed->arm_motor_speed);
	}
	if(motor_speed->left_motor_stop_brake != 0){
		if(motor_speed->left_motor_stop_brake)	pup_motor_stop(l_motor);
		else									pup_motor_brake(l_motor);
	}
	else{
		pup_motor_set_power(l_motor, motor_speed->left_motor_speed);
	}
	if(motor_speed->right_motor_stop_brake != 0){
		if(motor_speed->right_motor_stop_brake)	pup_motor_stop(r_motor);
		else									pup_motor_brake(r_motor);
	}
	else{
		pup_motor_set_power(r_motor, motor_speed->right_motor_speed);
	}
}

void reset_count_callback(const void * msgin)
{
    const raspike_uros_msg__msg__MotorResetMessage * reset_count = (const raspike_uros_msg__msg__MotorResetMessage *)msgin;

	if(reset_count->arm_motor_reset)	pup_motor_reset_count(a_motor);
	if(reset_count->left_motor_reset)	pup_motor_reset_count(l_motor);
	if(reset_count->right_motor_reset)	pup_motor_reset_count(r_motor);
}

void speaker_callback(const void * msgin)
{
    const raspike_uros_msg__msg__SpeakerMessage * speaker_tone_val = (const raspike_uros_msg__msg__SpeakerMessage *)msgin;

	temp_speaker_frequency = speaker_tone_val->tone;
    if (speaker_tone_val->tone <= 10 && speaker_tone_val->tone >= 1){
        play_speaker(speaker_tone_val->tone);
        speaker_play_duration = (speaker_tone_val->duration / 10);
        speaker_enabled = true;
        speaker_cnt = 0;
    }
}

void color_mode_callback(const void * msgin)
{
	const std_msgs__msg__Int8 * color_sensor_mode = (const std_msgs__msg__Int8 *)msgin;

	current_color_mode = color_sensor_mode->data;
}

void ultrasonic_mode_callback(const void * msgin)
{
	const std_msgs__msg__Int8 * ultrasonic_sensor_mode = (const std_msgs__msg__Int8 *)msgin;

	current_ultrasonic_mode = ultrasonic_sensor_mode->data;
}

void imu_init_callback(const void * msgin)
{
	const std_msgs__msg__Bool * imu_init = (const std_msgs__msg__Bool *)msgin;

	if(imu_init->data)	hub_imu_init();
}

/*
 *  メインタスク
 */
void 
uros_task(intptr_t exinf)
{
	syslog(LOG_NOTICE, "miro-ROS main task : start");

    /*get device pointers*/
    a_motor = pup_motor_get_device(PBIO_PORT_ID_A);
    r_motor = pup_motor_get_device(PBIO_PORT_ID_B);
    l_motor = pup_motor_get_device(PBIO_PORT_ID_E);
    col = pup_color_sensor_get_device(PBIO_PORT_ID_C); 
    ult = pup_ultrasonic_sensor_get_device(PBIO_PORT_ID_D);

	// Set transports
	set_microros_transports(UROS_PORTID);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// Create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "spike_state_node", "", &support));

	// Create publisher
	RCCHECK(rclc_publisher_init_best_effort(&button_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, ButtonStatusMessage), "spike_button_status"));
	RCCHECK(rclc_publisher_init_best_effort(&dev_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, SpikeDevStatusMessage), "spike_device_status"));
	RCCHECK(rclc_publisher_init_best_effort(&power_status_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, SpikePowerStatusMessage), "spike_power_status"));

	// Create subscriber
	RCCHECK(rclc_subscription_init_best_effort(&motor_speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, MotorSpeedMessage), "wheel_motor_speeds"));
	RCCHECK(rclc_subscription_init_default(&reset_count_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, MotorResetMessage), "motor_reset_count"));
    RCCHECK(rclc_subscription_init_default(&speaker_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, SpeakerMessage), "speaker_tone"));
	RCCHECK(rclc_subscription_init_best_effort(&color_mode_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "color_sensor_mode"));
	RCCHECK(rclc_subscription_init_default(&ultrasonic_mode_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "ultrasonic_sensor_mode"));
	RCCHECK(rclc_subscription_init_default(&imu_init_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "imu_init"));

	// Create timer,
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback));

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &motor_speed_subscriber, &motor_speed, &motor_speed_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &reset_count_subscriber, &reset_count, &reset_count_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &speaker_subscriber, &speaker_tone_val, &speaker_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &color_mode_subscriber, &color_sensor_mode, &color_mode_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &ultrasonic_mode_subscriber, &ultrasonic_sensor_mode, &ultrasonic_mode_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &imu_init_subscriber, &imu_init, &imu_init_callback, ON_NEW_DATA));


    syslog(LOG_NOTICE, "miro-ROS main task : init done.");

	pup_color_sensor_detectable_colors(7, raspike_rt_detectable_color);    //EV3-RTのAPIに合わせる場合はpup_color_sensor_detectable_colors(8, detectable_color_for_EV3)

    /*set up right motor*/
    for(int i = 0; i < 10; i++)
    {
        bool reset_count = true;
        r_err = pup_motor_setup(r_motor, PUP_DIRECTION_CLOCKWISE, true);
        pup_motor_reset_count(r_motor);
        if(r_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }

    /*set up left motor*/
    for(int i = 0; i < 10; i++){
        l_err = pup_motor_setup(l_motor, PUP_DIRECTION_COUNTERCLOCKWISE, true);
        pup_motor_reset_count(l_motor);
        if(l_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }

    /*set up arm motor*/
    for(int i = 0; i < 10; i++){
        a_err = pup_motor_setup(a_motor, PUP_DIRECTION_CLOCKWISE, true);
        pup_motor_reset_count(a_motor);
        if(a_err != PBIO_ERROR_AGAIN){
            break;
        }
        // Set up failed -> Wait 1s and ry one more
        dly_tsk(1000000);
    }

    hub_imu_init();
    hub_speaker_set_volume(100);

	send_color_value_1 = 0;
    send_color_value_2 = 0;
    send_color_value_3 = 0;
    send_color_mode_id = 0;
    send_ultrasonic_mode_id = 0;
    send_ultrasonic_value = 0;
    current_color_mode = 0;
    current_ultrasonic_mode = 0;

	syslog(LOG_NOTICE, "SPIKE init done.");

    hub_display_off();
    hub_display_orientation(PBIO_SIDE_TOP);
    pup_ultrasonic_sensor_light_set(ult, 60, 60, 60, 60);
    hub_display_image(text_ET);         //ディスプレイ表示
    hub_light_off();

	while(1){
		rclc_executor_spin(&executor);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&button_status_publisher, &node));
	RCCHECK(rcl_publisher_fini(&dev_status_publisher, &node));
	RCCHECK(rcl_publisher_fini(&power_status_publisher, &node));	
	RCCHECK(rcl_subscription_fini(&motor_speed_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&reset_count_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&color_mode_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&ultrasonic_mode_subscriber, &node));
    RCCHECK(rcl_subscription_fini(&speaker_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&imu_init_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
