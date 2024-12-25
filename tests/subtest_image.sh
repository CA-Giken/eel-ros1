#!/bin/bash

# Image型データのパブリッシュ(黒い画像データ)
BYTES_DATA=$(python3 -c "import sys; sys.stdout.buffer.write(bytes([0] * (640 * 480 * 3)))")

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
  data: ${BYTES_DATA}
}"