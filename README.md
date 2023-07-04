# ROS通信によるETロボコン走行体の制御アプリケーション開発用プラットフォーム

## 動作環境
- SPIKE側
    - spike-rt(OS)とmicro-ROS_ASP3を使用
        - [micro-ROS_ASP3](https://github.com/exshonda/micro-ROS_ASP3)
        - [spike-rt](https://github.com/spike-rt/spike-rt)
- rasberryPi側
    - rasberryPi OS(64bit)
    - ROS2 Humble

- 動作確認済みのバージョン
    - micro-ROS_ASP3
        - コミット識別番号：3a306729a797d0f4976daab50c5698acffe38a12
        - [Git Hub](https://github.com/exshonda/micro-ROS_ASP3/tree/3a306729a797d0f4976daab50c5698acffe38a12)
    - spike-rt
        - コミット識別番号：f6724115b0ef8c8367a760eaec2840089e6b4e55
        - [Git Hub](https://github.com/spike-rt/spike-rt/tree/f6724115b0ef8c8367a760eaec2840089e6b4e55)

## 使用方法
- linetrace_sample
    - ROS2アプリケーションのサンプル
    - ライントレースプログラム
    - rasberryPi上で実行
    - `<ROS2ワークスペース>\src`に置いてビルド
    - `$ ros2 run linetrace_sample lt_sample_node`で実行

- raspike_uros_msg
    - メッセージ型定義用パッケージ
    - SPIKEとrasberryPiの両方で使用
        - SPIKE：`micro-ROS_ASP3\external\primehub\firmware\mcu_ws`に置く
        - rasberryPi：`<ROS2ワークスペース>\src`に置く

- ros2_raspike_rt
    - アプリ開発用パッケージ
        - Pythonによるアプリ開発が可能
        - APIは[APIリファレンス](./ros2_raspike_rt/API_REFERENCE.md)を参照
        - アプリケーションの裏でROS2プログラムが稼働する
    - appNodeクラス内の`app_timer()`に処理を記述する
        - 周期的に呼ばれる
        - サンプルとしてライントレースプログラムが書かれている
    - `$ ros2 run ros2_raspike_rt rpi_ros2_node`で実行

- uros_raspike-rt
    - SPIKE側で動作するuROSパッケージ
    - `micro-ROS_ASP3\spike-rt`に置く
    - SPIKEをDFUモードにして書き込む

## カスタムメッセージ型 メッセージ内容
- SPIKE(uRPS) → rasberryPi(ROS2)<BR>
![to_rpi_message](./imgs/to_rpi_msg_contents.png)

- rasberryPi(ROS2) → SPIKE(uROS) <BR>
![to_spike_message](./imgs/to_spike_msg_contents.png)

## 設計メモ
- uROS(SPIKE)側のQoSについて
    - uROS側で10ms周期で送信するトピックのQoSをRELIABLEにすると，uROS側のサブスクライバーがデータをドロップすることを確認．
        - uROSはシングルタスクで動いている．
        - uROSのパブリッシャー(RELIABLE)はトピックをパブリッシュするとack待ちを行う．
        - ack待ちの間は，uROSのサブスクライバーは実行を待たされる．
        - 待たされている間に次のデータが来ると，前のデータをドロップする．
    - そのため，周期送信を行うトピックのQoSはBEST-EFFORT(ackを返さない)を使用した．
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
    - コマンドの値はrasPike環境に揃えている．
    
