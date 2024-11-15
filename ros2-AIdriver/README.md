# ros2-AIdriver

## 概要
- RasPike-ROS + TensolFlowにより自動走行を行うプログラム
- 使用方法はWikiに記載
    - [Wiki: ros2-AIdriver 環境構築方法・使用方法](https://github.com/Hiyama1026/raspike-ros/wiki/ros2%E2%80%90AIdriver)

## 動作確認済み環境
- Raspberry Pi 4
    - Ubuntu22.04LTS (64bit)
    - Raspberry Pi Camera v2
    - ROS 2 Humble
- ホストPC
    - Windowns11
    - WSL2 (Ubuntu22.04)
    - NVIDIA GPU
        - Windows x86_64 Driver Version >= 452.39
    - CUDA 11.8
    - tensorflow 2.14

## ファイル概要
- ros2_ai_driver
    - ROS 2パッケージ
    - 機械学習によりモータ回転速度を推論し，ROS 2トピックとしてパブリッシュ
- use_model
    - ライントレースを行うモデル
    - 円形のラインを時計回りにトレースするように学習したモデル
- exe_trainning.sh
    - 学習操作を自動化するスクリプト
    - ホストPCで使用
