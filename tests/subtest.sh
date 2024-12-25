#!/bin/bash

# Bool型データのパブリッシュ
rostopic pub /eel_subtest/bool std_msgs/Bool "data: true" -1

# Int32型データのパブリッシュ
rostopic pub /eel_subtest/int32 std_msgs/Int32 "data: 42" -1

# Int64型データのパブリッシュ
rostopic pub /eel_subtest/int64 std_msgs/Int64 "data: 9223372036854775807" -1

# Float32型データのパブリッシュ
rostopic pub /eel_subtest/float32 std_msgs/Float32 "data: 3.14159" -1

# Float64型データのパブリッシュ
rostopic pub /eel_subtest/float64 std_msgs/Float64 "data: 3.141592653589793" -1

# String型データのパブリッシュ
rostopic pub /eel_subtest/string std_msgs/String "data: 'Hello ROS!'" -1

# Transform型データのパブリッシュ
rostopic pub /eel_subtest/transform geometry_msgs/Transform "{
  translation: {
    x: 1.0,
    y: 2.0,
    z: 3.0
  },
  rotation: {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0
  }
}" -1

# Pose型データのパブリッシュ
rostopic pub /eel_subtest/pose geometry_msgs/Pose "{
  position: {
    x: 1.0,
    y: 2.0,
    z: 3.0
  },
  orientation: {
    x: 0.0,
    y: 0.0,
    z: 0.0,
    w: 1.0
  }
}" -1

# Frameへのパブリッシュ
rostopic pub /eel_subtest/frame1/bool std_msgs/Bool "data: true" -1
rostopic pub /eel_subtest/frame2/bool std_msgs/Bool "data: true" -1
rostopic pub /eel_subtest/frame3/bool std_msgs/Bool "data: true" -1