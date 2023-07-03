# API Reference

## モーター
### 関数
- void [set_wheel_speed](#void-set_wheel_speedint8-wheel_left_speed-int8-wheel_right_speed)(int8 wheel_left_speed, int8 wheel_right_speed)
- void [set_left_motor_speed](#void-set_left_motor_speedint8-wheel_left_speed)(int8 wheel_left_speed)
- void [set_right_motor_speed](#void-set_right_motor_speedint8-wheel_right_speed)(int8 wheel_right_speed)
- void [set_arm_motor_speed](#void-set_arm_motor_speedint8-arm_speed)(int8 arm_speed)
- void [wheel_motor_stop](#void-wheel_motor_stopvoid)(void)
- void [right_motor_stop](#void-right_motor_stopvoid)(void)
- void [left_motor_stop](#void-left_motor_stopvoid)(void)
- void [arm_motor_stop](#void-arm_motor_stopvoid)(void)
- void [wheel_motor_brake](#void-wheel_motor_brakevoid)(void)
- void [right_motor_brake](#void-right_motor_brakevoid)(void)
- void [left_motor_brake](#void-left_motor_brakevoid)(void)
- void [arm_motor_brake](#void-arm_motor_brakevoid)(void)
- int32 [get_right_motor_count](#int32-get_right_motor_countvoid)(void)
- int32 [get_left_motor_count](#int32-get_left_motor_countvoid)(viod)
- int32 [get_arm_motor_count](#int32-get_arm_motor_countvoid)(viod)
- void [wheel_motor_reset_count](#void-wheel_motor_reset_countvoid)(void)
- void [right_motor_reset_count](#void-right_motor_reset_countvoid)(void)
- void [left_motor_reset_count](#void-left_motor_reset_countvoid)(void)
- void [arm_motor_reset_count](#void-arm_motor_reset_countvoid)(void)


### 関数詳解
#### void set_wheel_speed(int8 wheel_left_speed, int8 wheel_right_speed)
走行体の左右のホイールスピードを指定する．
- 引数
    - 左車輪モーター速度指令値，右車輪モーター速度指令値
- 戻り値
    - 無し

#### void set_left_motor_speed(int8 wheel_left_speed)
左車輪用モーターの速度を指定する．
- 引数
    - 左車輪モーター速度指令値
- 戻り値
    - 無し

#### void set_right_motor_speed(int8 wheel_right_speed)
右車輪用モーターの速度を指定する．
- 引数
    - 右車輪モーター速度指令値
- 戻り値
    - 無し

#### void set_arm_motor_speed(int8 arm_speed)
アームモーターの速度指令値を指定する．
- 引数
    - アームモーターの速度指令値
- 戻り値
    - なし

#### void wheel_motor_stop(void)
ホイールモーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void right_motor_stop(void)
右モーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void left_motor_stop(void)
左モーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void arm_motor_stop(void)
アームモーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void wheel_motor_brake(void)
ホイールモーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void right_motor_brake(void)
右モーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void left_motor_brake(void)
左モーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void arm_motor_brake(void)
アームモーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

#### int32 get_right_motor_count(void)
右モーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値

#### int32 get_left_motor_count(void)
左モーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値

#### int32 get_arm_motor_count(void)
アームモーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値

#### void wheel_motor_reset_count(void)
左右の車輪用モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void right_motor_reset_count(void)
右モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void left_motor_reset_count(void)
左モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

#### void arm_motor_reset_count(void)
アームモーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

