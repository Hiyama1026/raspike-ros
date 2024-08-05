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
    - ROS 2のインストールはraspike-rosの[README.md](../README.md)等を参照
    - ROS 2ワークスペースの作成は[付録](#参考2ros-2ワークスペースの作成方法)を参照
- サンプルプログラムをROS2ワークスペースにコピー
    ```bash
    cp -r raspike-ros/linetrace_sample [PATH-TO-ROS2_WS]    # ROS 2アプリパッケージ
    cp -r raspike-ros/raspike_uros_msg [PATH-TO-ROS2_WS]    # カスタムメッセージ型定義ファイル
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


