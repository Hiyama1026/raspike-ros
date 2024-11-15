#!/bin/bash

# 
JPEG_DIR="~/ros2_ws/log/jpg"
CSV_DIR="~/ros2_ws/log/log.csv"
RPI_ADDRESS="hiyama@192.168.11.180"
TRAIN_FILE="train_80x60.py"
PATH_TO_MODELFILE="~/ros2_ws/use_model/model_a/"


echo
echo "Check the configuration."
echo "========================"
echo "Raspberry Pi address: $RPI_ADDRESS"
echo "JPEG files dir: $JPEG_DIR"
echo "CSV file: $CSV_DIR"
echo "Model file dir: $PATH_TO_MODELFILE"
echo "Train script: $TRAIN_FILE"
echo "========================"
echo


read -p "Do you want to continue? [Y/n]: " -n 1 -r
echo

# デフォルト値を設定
REPLY=${REPLY:-y}

# 入力に基づいて処理を実行
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo
else
    echo "Train prosess stoped."
    exit 1
fi

echo "Copy JPEG files from $RPI_ADDRESS:$JPEG_DIR to ./"
echo "Copy CSV file from $RPI_ADDRESS:$CSV_DIR to ./"

rsync -rtv --progress -partial $RPI_ADDRESS:$JPEG_DIR $RPI_ADDRESS:$CSV_DIR ./

# メッセージを表示してユーザーからの入力を待つ
read -p "Do you want to delete the file $RPI_ADDRESS:$JPEG_DIR? [Y/n]: " -n 1 -r
echo

# デフォルト値を設定
REPLY=${REPLY:-y}

# 入力に基づいて処理を実行
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ssh $RPI_ADDRESS rm -rf $JPEG_DIR/*
    status=$?
    if [ $status -eq 0 ]; then
        echo "File $RPI_ADDRESS:$JPEG_DIR has been deleted."
    else
        echo "Failed to delete jepeg files."
    fi
else
    echo "File $RPI_ADDRESS:$JPEG_DIR has NOT been deleted."
fi

echo
echo "========================"
echo

# exe trainning
python $TRAIN_FILE ./ 40

status=$?
if [ $status -eq 0 ]; then
    echo 
else
    exit 1
fi

echo
echo "========================"
echo

echo "Cpoy model file to $RPI_ADDRESS:$PATH_TO_MODELFILE"
rsync -rtv --progress -partial ./model.tflite $RPI_ADDRESS:$PATH_TO_MODELFILE

