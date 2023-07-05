# uros_raspike_rtのバイナリファイルの書き込み
## ファイル情報
- asp.dfu：uros_raspike_rtのバイナリファイル
- pydfu.py：書き込み処理を行うプログラム
## 書き込み
- SPIKEをDFUモードにする
    - bluetooth(BT)ボタンを押したまま，hubとPCをUSBケーブルで接続
    - BTボタンを「ピンク色に点灯」→「虹色に点滅」になるまで長押し
- 下記のコマンドでasp.dfuをSPIKEに書き込む
    - pyusbが無い場合はインストールする
```bash
$ sudo python3 ./pydfu.py -u asp.dfu --vid 0x0694 --pid 0x0008
```
