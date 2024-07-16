# SPIKE側ファームウェア（micro-ROS）の書き込み
## 概要
- SPIKE側ファームウェアの書き込み用
- SPIKE側の環境構築は下記手順を実施するのみで良い

## 動作確認済み環境
- WSL2
    - Ubuntu22.04 LTSで動作確認済み
- Linux (ネイティブ)
    - Ubuntu22.04 LTSで動作確認済み
- Raspberry Pi OS
    - 2023-05-03リリース版で動作確認済み

## 使用方法
- SPIKEをDFUモードにする
    - bluetooth(BT)ボタンを押したまま，hubとPCをUSBケーブルで接続
    - BTボタンを「ピンク色に点灯」→「虹色に点滅」になるまで長押し
- pyusbをインストール（Raspberry Pi OS，または Linux (ネイティブ)の場合のみ）
    ```bash
    sudo pip3 install pyusb
    ```
- ディレクトリ移動
    ```bash
    cd ~/uros_raspike-rt/bin
    ```
- 下記のコマンドでファームウェアをSPIKEに書き込む
    - WSL2の場合
        ```bash
        ./dfu-write.sh win
        ```
    - Raspberry Pi OS，または Linux (ネイティブ)の場合
        ```bash
        ./dfu-write.sh lin
        ```