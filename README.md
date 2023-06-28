# asp_urosによるROSライントレースプログラム
- ETロボコン用走行体をROSでライントレースするプログラム

## 動作環境
- SPIKE側
    - micro-ROS_ASP3を使用
        - [micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)
        - [spike-rt](https://github.com/spike-rt/spike-rt)
- rasberryPi側
    - rasberryPi OS(64bit)
    - ROS2 Humble


## 使用方法
- linetrace_sample
    - rasberryPi側で動作するROS2パッケージ
    - `<ROS2ワークスペース>\src`に置いてビルド

- raspike_uros_msg
    - メッセージ型定義用パッケージ
    - SPIKEとrasberryPiの両方で使用
        - SPIKE：`micro-ROS_ASP3\external\primehub\firmware\mcu_ws`に置く
        - rasberryPi：`<ROS2ワークスペース>\src`に置く

- ros2_raspike_rt
    - アプリ開発用ROS2パッケージ
    - appNodeクラス内の`app_timer()`に処理を記述
        - 周期的に呼ばれる
        - ToDo：APIを用意する

- uros_raspike-rt
    - SPIKE側で動作するuROSパッケージ
    - `micro-ROS_ASP3\spike-rt`に置く