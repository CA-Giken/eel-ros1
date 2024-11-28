#!/bin/bash

# デフォルトのファイルのパス
default_file="$(rospack find eel_example)/config/config.yaml"

# ファイルの存在を確認
if [ -f "$specific_file" ]; then
    echo "Loading default parameters from $default_file"
    rosparam load "$default_file"
fi