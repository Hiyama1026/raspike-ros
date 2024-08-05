# RasPike-ROSでROS 2プログラミングを行う場合のガイド

## このドキュメントについて
- RasPike-ROSでは2種類のアプリケーションの開発方法が存在する
    - [ros2_raspike_rt](./ros2_raspike_rt)パッケージを使用して専用APIによるアプリ開発を行う方法
    - ROS 2 APIを直接扱い，ROS2プログラミングによるアプリケーション開発を行う方法
- 本ドキュメントは後者の開発方法において，トピックやカスタムメッセージ型の扱い方などを示したものである

## 前提
- RasPike-ROSの環境構築が完了している事が前提条件
- ユーザはRaspberry Pi（もしくはROS2に接続可能なLinux PC）上でROS2アプリ開発を行う
    - SPIKE上のmicro-ROS(uROS)プログラムの通信相手となるプログラムを開発する
        - uros_raspike-rt(bin/asp.bin)の通信相手

## 目次
- [トピックの扱い方](#トピックの扱い方)
- [カスタムメッセージ型の扱い方](#カスタムメッセージ型の扱い方)
- [センサモード](#センサモード)
- [カラーコード](#カラーコード)
- [ボタンコマンド](#ボタンコマンド)
- [スピーカの使い方](#スピーカの使い方)
- [ros2アプリケーションを新規に作成する方法(参考)](#参考ros2アプリケーションを新規に作成する方法)

<br>
<br>
<br>
<br>
<br>


## トピックの扱い方
- RasPike-ROSでは全9種類のトピックを扱う
    - uROS(SPIKE)→ROS2(ユーザアプリ)のトピック：3種類
    - ROS2(ユーザアプリ)→uROS(SPIKE)のトピック：6種類

- RasPike-ROSではROS 2の標準メッセージ型(Int8, Bool)とカスタムメッセージ型を使用している
    - カスタムメッセージ型のパッケージ名は**raspike_uros_msg**である

- 各トピックにおけるトピック名やメッセージ型名の関係は下記に示す通りである

|送信方向|トピック名|メッセージ型名|QoS|トピックの概要|
|---|---|---|---|---|
|uROS→ROS2|spike_device_status|SpikeDevStatusMessage|best-effort|・センサ値やエンコーダ値等を格納する<br>・10ms周期でパブリッシュされる|
|uROS→ROS2|spike_button_status|ButtonStatusMessage|best-effort|・Hub内蔵ボタンの押下情報を格納する<br>・押下状態が変化した時にパブリッシュされる|
|uROS→ROS2|spike_power_status|SpikePowerStatusMessage|best-effort|・Hub内蔵バッテリーの情報を格納する<br>・100ms周期で送信される|
|ROS2→uROS|wheel_motor_speeds|MotorSpeedMessage|best-effort|・各モータに対する回転速度や停止の指令値を格納する<br>・ユーザが必要に応じてパブリッシュを行う|
|ROS2→uROS|motor_reset_count|MotorResetMessage|reliable|・各モータに対するエンコーダ値のリセット指令値を格納する<br>・エンコーダ値をリセットしたい時にユーザがパブリッシュを行う|
|ROS2→uROS|speaker_tone|SpeakerMessage|reliable|・Hub内蔵スピーカへの指令値を格納する<br>・スピーカを使用したい時にユーザがパブリッシュを行う|
|ROS2→uROS|color_sensor_mode|Int8<br>(標準メッセージ型)|best-effort|・カラーセンサモード(後述)の指令値を格納する<br>・モードを変更したい場合にユーザがパブリッシュを行う|
|ROS2→uROS|ultrasonic_sensor_mode|Int8<br>(標準メッセージ型)|reliable|・距離センサモード(後述)の指令値を格納する<br>・モードを変更したい場合にユーザがパブリッシュを行う|
|ROS2→uROS|imu_init|Bool<br>(標準メッセージ型)|reliable|・imuの初期化指令値を格納する<br>・リセットが必要な時にユーザがパブリッシュを行う|

- ユーザは上記のトピックを扱ってSPIKE側に指令を出す
    - トピックのQoSは表に記したものに揃える必要がある
    - ユーザはuROS→ROS2のトピックの受信処理と，ROS2→uROSのトピックの送信処理を記述する
        - uROS(SPIKE)からサブスクライブしたセンサ値を元にアクチュエータへの指令値を計算し，指令値をパブリッシュする
    - 記述例は[後述](#カスタムメッセージ型の使用例)

### パブリッシャ・サブスクライバの初期化記述例
- ユーザアプリ（Python）における全トピックのパブリッシャ・サブスクライバの初期化記述を以下に示す
    ```
    from rclpy.qos import QoSProfile    # インポート

    def __init__(self):
        qos_profile = QoSProfile(depth=10, reliability=2)   # BEST_EFFORTの設定のため
        
        # パブリッシャーの生成
        create_publisher(MotorSpeedMessage, "wheel_motor_speeds", qos_profile)
        create_publisher(MotorResetMessage, "motor_reset_count", 10)
        create_publisher(SpeakerMessage, "speaker_tone", 10)
        create_publisher(Int8, "color_sensor_mode", qos_profile)
        create_publisher(Int8, "ultrasonic_sensor_mode", 10)
        create_publisher(Bool, "imu_init", 10)

        # サブスクライバーの生成
        create_subscription(SpikeDevStatusMessage, "spike_device_status", [コールバック], qos_profile)
        create_subscription(ButtonStatusMessage, "spike_button_status", [コールバック], qos_profile)
        create_subscription(SpikePowerStatusMessage, "spike_power_status", [コールバック], qos_profile)
    ```
<br>
<br>

## カスタムメッセージ型の扱い方
- メッセージ型定義ファイルは[raspike_uros_msg/msg](./raspike_uros_msg/msg)にある
    - 使用方法はROS 2の一般的な方法と同じ
    - メッセージ型内の各変数の役割は[カスタムメッセージ型の仕様表](#カスタムメッセージ型の仕様表)に示す通り

- メッセージ型をアプリケーション側でインポートする
    - 下記の[カスタムメッセージ型の仕様表](#カスタムメッセージ型の仕様表)で示したメッセージ型をインポートする
    - Pythonでの記述例
    ```
    from std_msgs.msg import Int8
    from std_msgs.msg import Bool
    from raspike_uros_msg.msg import MotorSpeedMessage
    from raspike_uros_msg.msg import MotorResetMessage
    from raspike_uros_msg.msg import SpeakerMessage
    from raspike_uros_msg.msg import SpikeDevStatusMessage
    from raspike_uros_msg.msg import ButtonStatusMessage
    from raspike_uros_msg.msg import SpikePowerStatusMessage
    ```

### カスタムメッセージ型の仕様表
- SPIKE(uRPS) → rasberryPi(ROS 2)<BR>
![to_rpi_message](./img/to_rpi_msg_contents.png)

- rasberryPi(ROS 2) → SPIKE(uROS) <BR>
![to_spike_message](./img/to_spike_msg_contents.png)

### カスタムメッセージ型の使用例
- モータの回転速度を指定(Python)
    ```
    def __init__(self):
        # パブリッシャーの生成
        self.motor_speed_publisher = self.create_publisher(MotorSpeedMessage, "wheel_motor_speeds", qos_profile)
    
    def timer_on_tick(self):    # 周期関数
       # メッセージの生成
        motor_speed = MotorSpeedMessage()

        motor_speed.right_motor_speed = 50
        motor_speed.left_motor_speed = 50
        motor_speed.arm_motor_speed = 0
        # メッセージのパブリッシュ
        self.motor_speed_publisher.publish(motor_speed) 
    ```

- Hun内蔵バッテリー情報の取得
    ```
    def __init__(self):
        self.rev_hub_volt = 0
        self.rev_hub_current = 0

        # サブスクライバの生成
        self.hub_status_subscription = self.create_subscription(SpikePowerStatusMessage, "spike_power_status", self.hub_status_on_subscribe, qos_profile)

    def hub_status_on_subscribe(self, hub_status):  # サブスクライバコールバック関数
        self.rev_hub_volt = hub_status.voltage
        self.rev_hub_current = hub_status.current
    ```

<br>
<br>

## センサモード
- カラーセンサセンサと距離センサはモードを切り替えると取得値の種類が変化する
    - カラーセンサ：全4モード
    - 距離センサ：全2モード
- センサモードの切り替えには若干の時間がかかるため，切り替わりが完了したことをユーザ側で確認する必要がある
    - トピック「spike_device_status」に含まれる下記の変数が，それぞれ現在のセンサモードを格納している
        - カラーセンサ：color_mode_id
        - 距離センサ：ultrasonic_mode_id

- 各モードの内容とカスタムメッセージ型の変数に格納される情報の詳細を以下に示す
    - カラーセンサ

    |モード|取得データ|メッセージ型とデータ内容の関係|
    |---|---|---|
    |0|無し（初期状態）|---|
    |1|ambient値|send_color_value_1=ambient値<br>send_color_value_2=0<br>send_color_value_3=0|
    |2|カラーコード（後述）|send_color_value_1=カラーコード<br>send_color_value_2=0<br>send_color_value_3=0|
    |3|reflection値|send_color_value_1=reflection値<br>send_color_value_2=0<br>send_color_value_3=0|
    |4|RGB値|send_color_value_1=R<br>send_color_value_2=G<br>send_color_value_3=B|

    - 距離センサ

    |モード|取得データ|メッセージ型とデータ内容の関係|
    |---|---|---|
    |0|無し（初期状態）|---|    
    |1|距離|ultrasonic_sensor=距離|    
    |2|presemce値|ultrasonic_sensor=presemce値|

<br>
<br>

## カラーコード
- カラーコードの対応表を以下に示す

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

<br>
<br>

## ボタンコマンド
- ボタンの押下状態を示すコマンドを以下に示す
    - コマンド値は加算方式
    - LEFTとRIGHTを同時に押す→3 (``0b00000011``)

|button|code|
|---|---|
|LEFT|+1 (``0b00000001``)|
|RIGHT|+2 (``0b00000010``)|
|CENTER|+16 (``0b00010000``)|

- bluetoothボタン
    - ButtonStatusMessageのtouch_sensorに2048が入る


<br>
<br>

## スピーカの使い方
- 駆動時間は10ms単位で指定（``duration``）
- トーンは10段階で指定する
- 使用例 : [go_straight_pubsub.cpp](./sample/go_straight_cpp/src/go_straight_pubsub.cpp/#L60-L61)

|set value|tone|
|---|---|
|1|C4|
|2|D4|
|3|E4|
|4|F4|
|5|G4|
|6|A4|
|7|B4|
|8|C5|
|9|D5|
|10|E5|


## 参考：ROS2アプリケーションを新規に作成する方法
1. 下記のコマンドでROS2パッケージを新規作成
    ```bash
    $ cd ~/ros2_ws/src
    $ ros2 pkg create --build-type ament_python [パッケージ名] --dependencies rclpy
    ```

1. setpu.pyを以下のように編集
    ```
    entry_points={
        'console_scripts': [
            '[ノード名] = [パッケージ名].[ノード名]:main'
            '[ノード名] = [パッケージ名].[ノード名]:main'
        ],
    },
    ```

1. アプリケーションを作成する
    - ファイルを作成
        ```bash
        $ cd src/[パッケージ名]/[パッケージ名]
        $ touch [ノード名].py
        ```
    - `[ノード名].py`に処理を記述
        - lt_sample_node.pyの中身をコピーする場合はノード名(lt_sample_node)を変更する
            ```
            class linetracerNode(Node):
                # 初期化
                def __init__(self):
                    super().__init__("[ノード名]")
            ```

1. ビルド・セットアップ・実行
    - 下記のコマンドを実行
    ```
    $ cd ~/ros2_ws
    $ colcon build
    $ . install/setup.bash
    $ ros2 run [パッケージ名] [ノード名]
    ```