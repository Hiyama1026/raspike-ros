# API Reference

## モジュール
- [モーター](#モーター)
- [カラーセンサー](#カラーセンサー)
- [超音波センサー](#超音波センサー)
- [IMU](#imu)
- [ボタン](#ボタン)
- [スピーカー](#スピーカー)
- [Hub Power Status](#hub-power-status)

## モーター
### <関数>
- void [motor.set_wheel_speed](#void-motorset_wheel_speedint8-wheel_left_speed-int8-wheel_right_speed)(int8 wheel_left_speed, int8 wheel_right_speed)
- void [motor.set_left_motor_speed](#void-motorset_left_motor_speedint8-wheel_left_speed)(int8 wheel_left_speed)
- void [motor.set_right_motor_speed](#void-motorset_right_motor_speedint8-wheel_right_speed)(int8 wheel_right_speed)
- void [motor.set_arm_motor_speed](#void-motorset_arm_motor_speedint8-arm_speed)(int8 arm_speed)
- void [motor.wheel_motor_stop](#void-motorwheel_motor_stopvoid)(void)
- void [motor.right_motor_stop](#void-motorright_motor_stopvoid)(void)
- void [motor.left_motor_stop](#void-motorleft_motor_stopvoid)(void)
- void [motor.arm_motor_stop](#void-motorarm_motor_stopvoid)(void)
- void [motor.wheel_motor_brake](#void-motorwheel_motor_brakevoid)(void)
- void [motor.right_motor_brake](#void-motorright_motor_brakevoid)(void)
- void [motor.left_motor_brake](#void-motorleft_motor_brakevoid)(void)
- void [motor.arm_motor_brake](#void-motorarm_motor_brakevoid)(void)
- int32 [motor.get_right_motor_count](#int32-motorget_right_motor_countvoid)(void)
- int32 [motor.get_left_motor_count](#int32-motorget_left_motor_countvoid)(viod)
- int32 [motor.get_arm_motor_count](#int32-motorget_arm_motor_countvoid)(viod)
- void [motor.wheel_motor_reset_count](#void-motorwheel_motor_reset_countvoid)(void)
- void [motor.right_motor_reset_count](#void-motorright_motor_reset_countvoid)(void)
- void [motor.left_motor_reset_count](#void-motorleft_motor_reset_countvoid)(void)
- void [motor.arm_motor_reset_count](#void-motorarm_motor_reset_countvoid)(void)


### <関数詳解>
### void motor.set_wheel_speed(int8 wheel_left_speed, int8 wheel_right_speed)
走行体の左右のホイールスピードを指定する．
- 引数
    - 左車輪モーター速度指令値，右車輪モーター速度指令値
- 戻り値
    - 無し

### void motor.set_left_motor_speed(int8 wheel_left_speed)
左車輪用モーターの速度を指定する．
- 引数
    - 左車輪モーター速度指令値
- 戻り値
    - 無し

### void motor.set_right_motor_speed(int8 wheel_right_speed)
右車輪用モーターの速度を指定する．
- 引数
    - 右車輪モーター速度指令値
- 戻り値
    - 無し

### void motor.set_arm_motor_speed(int8 arm_speed)
アームモーターの速度指令値を指定する．
- 引数
    - アームモーターの速度指令値
- 戻り値
    - なし

### void motor.wheel_motor_stop(void)
ホイールモーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.right_motor_stop(void)
右モーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.left_motor_stop(void)
左モーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.arm_motor_stop(void)
アームモーターをストップする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.wheel_motor_brake(void)
ホイールモーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.right_motor_brake(void)
右モーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.left_motor_brake(void)
左モーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.arm_motor_brake(void)
アームモーターをブレーキする．
- 引数
    - 無し
- 戻り値
    - 無し

### int32 motor.get_right_motor_count(void)
右モーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値[°]

### int32 motor.get_left_motor_count(void)
左モーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値[°]

### int32 motor.get_arm_motor_count(void)
アームモーターのエンコーダー値を取得する．
- 引数
    - 無し
- 戻り値
    - エンコーダーの値[°]

### void motor.wheel_motor_reset_count(void)
左右の車輪用モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.right_motor_reset_count(void)
右モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.left_motor_reset_count(void)
左モーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し

### void motor.arm_motor_reset_count(void)
アームモーターのエンコーダ値をリセットする．
- 引数
    - 無し
- 戻り値
    - 無し


## カラーセンサー

### <カラーモード>
|command|color mode|
|---|---|
|0|color sensor off|
|1|get ambient|
|2|get color_code|
|3|get reflection|
|4|get rgb value|

### <カラーコード>
|color code|color|
|---|---|
|0|NONE|
|1|RED|
|2|YELLOW|
|3|GREEN|
|4|BLUE|
|5|WHILE|
|6|BRACK|
|-2|err|

### <関数>
- void [color_sensor.set_color_mode](#void-color_sensorset_color_modeint8-color_mode)(int8 color_mode)
- int8 [color_sensor.get_color_mode](#int8-color_sensorget_color_modevoid)(void)
- int16 [color_sensor.get_ambient](#int16-color_sensorget_ambientvoid)(void)
- int16 [color_sensor.get_color_code](#int16-color_sensorget_color_codevoid)(void)
- int16 [color_sensor.get_reflection](#int16-color_sensorget_reflectionvoid)(void)
- int16 [color_sensor.get_rgb](#int16-color_sensorget_rgbvoid)(void)
- int16 [color_sensor.get_rgb_r](#int16-color_sensorget_rgb_rvoid)(void)
- int16 [color_sensor.get_rgb_g](#int16-color_sensorget_rgb_gvoid)(void)
- int16 [color_sensor.get_rgb_b](#int16-color_sensorget_rgb_bvoid)(viod)


### <関数詳解>
### void color_sensor.set_color_mode(int8 color_mode)
カラーセンサーのモードを指定する．
- 引数
    - カラーモード（0~4）
- 戻り値
    - 無し
- **注意**
    - モードの切り替えには多少の時間がかかる．
    - モード切り替えの際，切り替えが完了する前にセンサ値取得関数が呼ばれた場合はセンサ値取得関数がエラーを返す．

### int8 color_sensor.get_color_mode(void)
現在のカラーモードを取得する．<BR>
- 引数
    - 無し
- 戻り値
    - カラーモード(0~4)

### int16 color_sensor.get_ambient(void)
アンビエント値を取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - アンビエント値,もしくはエラーコード(-1)

### int16 color_sensor.get_color_code(void)
色(カラーコード)を取得する．<BR>
カラーコードをEV3-RTと同じものにしたい場合は，uros.cの`raspike_rt_detectable_color`を`detectable_color_for_EV3`に変更する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - カラーコード,もしくはエラーコード(-1)

### int16 color_sensor.get_reflection(void)
リフレクション値を取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - どの程度反射しているか[%],もしくはエラーコード(-1)

### int16[3] color_sensor.get_rgb(void)
RGB値を取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - RGB値を格納した配列,もしくはエラーコード(-1)
    - rgb[0]=R, rgb[1]=G, rgb[2]=B

### int16 color_sensor.get_rgb_r(void)
RGB値のうち，Rを取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - 色合い(R),もしくはエラーコード(-1)

### int16 color_sensor.get_rgb_g(void)
RGB値のうち，Gを取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - 色合い(G),もしくはエラーコード(-1)

### int16 color_sensor.get_rgb_b(void)
RGB値のうち，Bを取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - 色合い(B),もしくはエラーコード(-1)


## 超音波センサー

### <超音波センサーモード>
|command|ultrasonic mode|
|---|---|
|0|ultrasonic sensor off|
|1|get distance|
|2|get presemce|

### <関数>
- void [ultrasonic_sensor.set_ultrasonic_mode](#void-ultrasonic_sensorset_ultrasonic_modeint8-ultrasonic_mode)(int8 ultrasonic_mode)
- int8 [ultrasonic_sensor.get_ultrasonic_mode](#int8-ultrasonic_sensorget_ultrasonic_modevoid)(void)
- int16 [ultrasonic_sensor.get_distance](#int16-ultrasonic_sensorget_distancevoid)(void)
- int16 [ultrasonic_sensor.get_presence](#int16-ultrasonic_sensorget_presencevoid)(void)

### <関数詳解>
### void ultrasonic_sensor.set_ultrasonic_mode(int8 ultrasonic_mode)
超音波センサーのモードを指定する．
- 引数
    - 超音波センサーモード（0~2）
- 戻り値
    - 無し
- **注意**
    - モードの切り替えには多少の時間がかかる．
    - モード切り替えの際，切り替えが完了する前にセンサ値取得関数が呼ばれた場合はセンサ値取得関数がエラーを返す．

### int8 ultrasonic_sensor.get_ultrasonic_mode(void)
現在の超音波センサーのモードを取得する．
- 引数
    - 無し
- 戻り値
    - 超音波センサーモード(0~2)

### int16 ultrasonic_sensor.get_distance(void)
障害物までの距離を取得する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - 距離[cm]

### int16 ultrasonic_sensor.get_presence(void)
付近に他の超音波センサーがあるかを検知する．<BR>
モードが切り替わる前に呼ばれた場合はエラーコードとして **-1** を返す．
- 引数
    - 無し
- 戻り値
    - 超音波センサーが存在するか
        - 1 → 存在する
        - 2 → 存在しない

## IMU

### <関数>
- void [imu.init](#void-imuinitvoid)(void)
- float32 [imu.get_x_angular_velocity](#float32-imuget_x_angular_velocityvoid)(void)


### <関数詳解>
### void imu.init(void)
IMUを初期化する．
- 引数
    - 無し
- 戻り値
    - 無し

### float32 imu.get_x_angular_velocity(void)
X軸方向の角速度を取得する．
- 引数
    - 無し
- 戻り値
    - X軸方向の角速度[°/s]


## ボタン

### ボタンコマンド
|button|command|
|---|---|
|LEFT|cmd += 1|
|RIGHT|cmd += 2|
|CENTER|cmd += 16|
|BLUETOOTH|bt_cmd = 2048|

### <関数>
- bool [button.is_center_pressed](#bool-buttonis_center_pressedvoid)(void)
- bool [button.is_left_pressed](#bool-buttonis_left_pressedvoid)(void)
- bool [button.is_right_pressed](#bool-buttonis_right_pressedvoid)(void)
- bool [button.is_bluetooth_pressed](#bool-buttonis_bluetooth_pressedvoid)(void)
- int8 [button.get_pressed_button_command](#int8-buttonget_pressed_button_commandvoid)(void)

### <関数詳解>
### bool button.is_center_pressed(void)
hubのセンターボタンが押されているかを返す．
- 引数
    - 無し
- 戻り値
    - True or False

### bool button.is_left_pressed(void)
hubの左ボタンが押されているかを返す．
- 引数
    - 無し
- 戻り値
    - True or False

### bool button.is_right_pressed(void)
hubの右ボタンが押されているかを返す．
- 引数
    - 無し
- 戻り値
    - True or False

### bool button.is_bluetooth_pressed(void)
hubのBluetoothボタンが押されているかを返す．
- 引数
    - 無し
- 戻り値
    - True or False

### int8 button.get_pressed_button_command(void)
現在押されているボタンのコマンドを返す．
- 引数
    - 無し
- 戻り値
    - ボタンコマンド

## スピーカー
### <関数>
- void [speaker.play_tone](#void-speakerplay_toneint8-tone)(int8 tone)

### <関数詳解>
### void speaker.play_tone(int8 tone)
指定したトーンでスピーカーを30ms間鳴らす．<Br>
トーンは1～10の10段階で指定できる．<Br>
- 引数
    - トーン(1~10)
- 戻り値
    - 無し
- **注意**
    - スピーカーが音を鳴らしている間は，センサー・アクチュエーターのステータス更新が停止する．

## Hub Power Status
### <関数>
- int16 [power.get_hub_voltage](#int16-powerget_hub_voltagevoid)(void)
- int16 [power.get_hub_current](#int16-powerget_hub_currentvoid)(void)

### <関数詳解>
### int16 power.get_hub_voltage(void)
hubの電圧を取得する．
- 引数
    - 無し
- 戻り値
    - 電圧[mV]

### int16 power.get_hub_current(void)
hubの電流を取得する．
- 引数
    - 無し
- 戻り値
    - 電流[mA]
