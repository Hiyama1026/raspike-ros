# ETロボコン走行体用ROSライントレースプログラム
- このパッケージは`[ROS2用ワークスペース]/src`に置くことで使用できる
- カスタムメッセージ型定義ファイルであるraspike_uros_msgも一緒にROS 2用ワークスペースに置く必要がある

## ファイル構成
- `src/linetrace_sample/setup.py`
    - ROS2のセットアップファイル
- `src/linetrace_sample/linetrace_sample/lt_sample_node.py`
    - ROS2アプリケーション
        - パッケージ名：linetrace_sample
        - ノード名：lt_sample_node
    - ETロボコン走行体をライントレースさせるプログラム

## 実行
- ROS 2のインストールとROS 2のワークスペースが作成済みである事が前提
    - ROS 2のインストールはuros_raspike-rtの[README.md](../README.md)等を参照
    - ROS 2ワークスペースの作成は[付録](#参考2ros-2ワークスペースの作成方法)を参照
- サンプルプログラムをROS2ワークスペースにコピー
    ```bash
    cp -r uros_raspike-rt/linetrace_sample [PATH-TO-ROS2_WS]    # ROS 2アプリパッケージ
    cp -r uros_raspike-rt/raspike_uros_msg [PATH-TO-ROS2_WS]    # カスタムメッセージ型定義ファイル
    ```
- ROS2ワークスペースをビルド
    ```bash
    cd [PATH-TO-ROS2_WS]
    colcon build
    . install/setup.bash
    ```
- 下記のコマンドでノードを起動(実行)
    ```bash
    ros2 run linetrace_sample lt_sample_node    # ros2 run <パッケージ名> <ノード名>
    ```


## 参考（[ROS2_GUIDE.md](../ROS2_GUIDE.md)の抜粋）    
### 参考1：使用できるメッセージ型・パブリッシャー・サブスクライバー
(※)[rpi_ros2_node.py](../ros2_raspike_rt/ros2_raspike_rt/lib/rpi_ros2_node.py)を参考にすると良い<Br>
- 使用できるメッセージ型（インポート方法）
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
- 使用できるパブリッシャー・サブスクライバ
    - QoSの設定
    ```
    from rclpy.qos import QoSProfile    # インポート

    qos_profile = QoSProfile(depth=10, reliability=2)   # BEST_EFFORTの設定のため
    ```
    - パブリッシャー・サブスクライバの生成API一覧
    ```
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

### 参考2：ROS 2ワークスペースの作成方法
- ROS 2のインストールが完了していることが前提
- ディレクトリを作成してビルド
    ```
    mkdir ~/ros2_ws
    cd ~/ros2_ws
    colcon build
    ```

### 参考3：ROS2アプリケーションを新規に作成する方法
1. 下記のコマンドでROS2パッケージを新規作成
    ```bash
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python [パッケージ名] --dependencies rclpy
    ```

1. setpu.pyを以下のように編集
    ```
    entry_points={
        'console_scripts': [
            '[ノード名] = [パッケージ名].[ノード名]:main'
        ],
    },
    ```

1. アプリケーションを作成する
    - ファイルを作成
        ```bash
        cd src/[パッケージ名]/[パッケージ名]
        touch [ノード名].py
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
    cd ~/ros2_ws
    colcon build
    . install/setup.bash
    ros2 run [パッケージ名] [ノード名]
    ```