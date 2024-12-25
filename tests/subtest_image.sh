#!/bin/bash

# Image型データのパブリッシュ(黒い画像データ)
ENCODED_DATA=$(python3 -c '
import base64
dummy_data = bytes([0] * (640 * 480 * 3))
encoded = base64.b64encode(dummy_data).decode()
print(encoded)
')

rostopic pub -1 /eel_subtest/image sensor_msgs/Image "{
  header: {
    seq: 0,
    stamp: {secs: 0, nsecs: 0},
    frame_id: 'camera'
  },
  height: 480,
  width: 640,
  encoding: 'rgb8',
  is_bigendian: 0,
  step: 1920,
  data: '$(echo -n "$ENCODED_DATA" | base64 -d)'
}"