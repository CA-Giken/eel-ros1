#!/bin/bash

package_name=$1

# ファイルパス
specific_file="$(rospack find $package_name)/config/config.yaml"

# ファイルの存在を確認
if [ -f "$specific_file" ]; then
    echo "[CA] Loading parameters from $specific_file"
    rosparam load "$specific_file"
fi