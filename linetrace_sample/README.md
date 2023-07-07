# ETロボコン走行体用ROSライントレースプログラム
- このパッケージは`[ROS2用ワークスペース]\src`に置くことで使用できる

## ファイル構成
- `src\linetrace_sample\setup.py`
    - ROS2のセットアップファイル
- `src\linetrace_sample\linetrace_sample\lt_sample_node.py`
    - ROS2アプリケーション
        - パッケージ名：linetrace_sample
        - ノード名：lt_sample_node
    - ETロボコン走行体をライントレースさせるプログラム

## 実行
- ROS2ワークスペースをビルド
    ```bash
    $ colcon build
    $ . install/setup.bash
    ```
- 下記のコマンドでノードを起動(実行)
    ```bash
    $ ros2 run linetrace_sample lt_sample_node
    ```

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