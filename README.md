# ROS通信によるETロボコン走行体の制御アプリケーション開発用プラットフォーム（RasPike-ROS）
# 目次
- [概要](#概要)
- [動作確認済み環境](#動作確認済み環境)
- [システムの構成](#システムの構成)
- [環境構築方法](#環境構築方法)
- [使用できるパブリッシャー・サブスクライバー](#使用できるパブリッシャーサブスクライバー)
- [カスタムメッセージ型の仕様](#カスタムメッセージ型raspike_uros_msgの仕様)
- [ファイル構成](#ファイル構成)
- [免責](#免責)
- [付録](#付録設計メモ)


# 概要
- 本ソフトウェアはROS 2によりETロボコン用走行体(もしくはそれと構成が同じロボット)を制御するためのソフトウェアプラットホームである
    - リポジトリ名をuros_raspike-rtからリネーム（2024/08/01）
- 本ソフトウェアを使用することよりROS 2のプログラムによりETロボコン用走行体を制御することが可能である
- アプリ開発用APIを使用すれば，ROS 2の知識を有していなくてもアプリ開発が可能である
- 実行動画(Youtube)：[https://youtu.be/RoaVhumuqcQ](https://youtu.be/RoaVhumuqcQ)

# 動作確認済み環境

- SPIKE
   - micro-ROS(uROS)ファームウェア[uros_raspike-rt](./uros_raspike_rt)
        - SPIKE上で動作し，各種センサー値をpublishし，モータ制御値をsubscribeする
        - [spike-rt](https://github.com/spike-rt/spike-rt)と[micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)を使用
            - ファームウェアに変更を加えたい場合に限りインストールが必要

- Raspberry Pi 4
    - Raspberry Pi OS(64bit，**2023-05-03リリース版**)またはUbuntu22.04LTS
    - ROS 2 Humble
- 動作確認済みのバージョン
    - spike-rt
        - バージョン：[v0.2.0](https://github.com/spike-rt/spike-rt/tree/v0.2.0)
    - micro-ROS_ASP3（通常は使用しない）
        - コミットID：[dfe4cc40bade9aace0b047611e1c0ed6da1a5dc2](https://github.com/exshonda/micro-ROS_ASP3/tree/dfe4cc40bade9aace0b047611e1c0ed6da1a5dc2)


# システムの構成
- 2種類の方法でアプリを開発可能である．(両者を同時に使用した場合の動作は保証しない)

<Br>

1. **ROS 2 APIを直接扱い．ROS2プログラミングによりアプリケーションを開発する方法**
    - [ETロボコン走行体向けカスタムメッセージ型](#カスタムメッセージ型の仕様)を直接扱う
    - ガイドを[ROS2_GUIDE.md](./ROS2_GUIDE.md)に記載
    - サンプルプログラム : [linetrace_sample](./sample-pkgs/linetrace_sample/)
1. **アプリケーション開発用APIを使用**
    - 走行体制御アプリケーションの開発向けにカスタムメッセージをラップした専用APIを使用する方法．
    - [専用API仕様](./ros2_raspike_rt/API_REFERENCE.md)
        - 時間コールバック関数である`app_timer()`内にプログラムを記述する．
    - サンプルプログラム : [app_node.py](./ros2_raspike_rt/ros2_raspike_rt/app_node.py)

# 環境構築方法
- [RasPike-ROS環境構築Wiki](https://github.com/Hiyama1026/raspike-ros/wiki)に従い，環境構築を行う


# 使用できるパブリッシャー・サブスクライバー
- [linetrace_sample/README.md](./sample-pkgs/linetrace_sample/README.md/#参考1使用できるメッセージ型・パブリッシャー・サブスクライバー)を参照
    - 注意：QoSを揃える必要がある

# カスタムメッセージ型(raspike_uros_msg)の仕様
- SPIKE(uRPS) → Raspberry Pi(ROS 2)<BR>
![to_rpi_message](./img/to_rpi_msg_contents.png)

- Raspberry Pi(ROS 2) → SPIKE(uROS) <BR>
![to_spike_message](./img/to_spike_msg_contents.png)

# ファイル構成
- bin
    - asp.dfu
        - micro-ROS(uROS)ファームウェアのプリビルドバイナリ
        - SPIKEに書き込んで使用する
    - pydfu.py
        - SPIKEへの書き込みプログラム

- raspike_uros_msg
    - ETロボコン走行体用カスタムメッセージのメッセージ型定義用ROS 2パッケージ
    - SPIKEとRaspberry Piの両方で使用
        - SPIKE：`micro-ROS_ASP3/external/primehub/firmware/mcu_ws`に置く
        - Raspberry Pi：`<ROS 2ワークスペース>/src`に置く

- ros2_raspike_rt(アプリ開発用APIを使用する場合に使用するROS 2パッケージ)
    - `ros2_raspike_rt/ros2_raspike_rt/app_node.py`
        - アプリ開発用のファイル
            - **ユーザはこのファイルにアプリを記述する**
            - Pythonによるアプリ開発が可能
            - appNodeクラス内の`app_timer()`に処理を記述する
                - 周期的に呼ばれる
        - APIは[APIリファレンス](./ros2_raspike_rt/API_REFERENCE.md)を参照
    - `ros2_raspike_rt/ros2_raspike_rt/lib`フォルダ内のファイル
        - ROS 2処理に関するのライブラリファイル等    
        - `rpi_ros2_node.py`
            - uROSからのセンサ値の受信・app_node.pyで計算された指令値の送信を行うROS 2プログラム
                - app_node.pyからAPIを介して指令値が渡される
                - uros_raspike-rt(SPIKE)と通信する

- sample-pkgs
    - ETロボコン走行体向けのROS 2サンプルプログラム

- uros_raspike-rt
    - SPIKE側で動作するuROSパッケージのソースコード
    - [bin/asp.dfu](./bin/asp.dfu)のソース


# 免責
- 本ソフトウェアの利用により直接的または間接的に生じたいかなる損害に関しても，その責任を負わない．


# 付録：設計メモ
- uROS(SPIKE)側のQoSについて
    - uROS側で10ms周期で送信するトピックのQoSをRELIABLEにすると，uROS側のサブスクライバーがデータをドロップすることを確認．
        - uROSはシングルタスクで動いている．
        - uROSのパブリッシャー(RELIABLE)はトピックをパブリッシュするとack待ちを行う．
        - ack待ちの間は，uROSのサブスクライバーは実行を待たされる．
        - 待たされている間に次のデータが来ると，前のデータをドロップする．
    - そのため，周期送信を行うトピックのQoSはBEST-EFFORT(ackを返さない)を使用した．
        - 以下のメッセージはBEST-EFFORTで通信する．
            - モーターの速度指令（MotorSpeedMessage）
            - カラーセンサーのモード指令
            - PUPデバイスのステータス（SpikeDevStatusMessage）
            - HUBボタンのステータス（ButtonStatusMessage）
            - HUBの電源情報（SpikePowerStatusMessage）
- メッセージのグループ化について
    - ROSの通信ではトピック一つにつき，比較的大きいサイズの情報が付随する．
    - 全メッセージを個別のトピックに分けるとオーバーヘッドが大きすぎるため，いくつかのメッセージを一つのトピックにグループ化した．
    - グループ化の方法にはArray型とカスタムメッセージ型がある．
        - 今回はカスタムメッセージ型を用いる方法を採用した．
            - カスタムメッセージ型はArray型に比べてオーバーヘッドが小さいため．

- カラーコードについて
    - カラーコードはオリジナルのものを用意した．
    - カラーコードは[APIリファレンス](./ros2_raspike_rt/API_REFERENCE.md)を参照．
    - EV3-RTのAPIに揃えたい場合はuros.cの`raspike_rt_detectable_color`を`detectable_color_for_EV3`に変更する．
- ボタンコマンドについて
    - SPIKE(uros.c)は，押されているボタンに応じたコマンドを送信している.
    - コマンドの値はRasPike環境に揃えている．
