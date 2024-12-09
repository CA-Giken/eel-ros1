#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool, Int32, Int64, Float32, Float64, String
from geometry_msgs.msg import Transform, Pose
from sensor_msgs.msg import Image

def publish_test_data():
    rospy.init_node('test_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1Hz

    # パブリッシャーの設定
    bool_pub = rospy.Publisher('/eel_subtest/bool', Bool, queue_size=1)
    int32_pub = rospy.Publisher('/eel_subtest/int32', Int32, queue_size=1)
    int64_pub = rospy.Publisher('/eel_subtest/int64', Int64, queue_size=1)
    float32_pub = rospy.Publisher('/eel_subtest/float32', Float32, queue_size=1)
    float64_pub = rospy.Publisher('/eel_subtest/float64', Float64, queue_size=1)
    string_pub = rospy.Publisher('/eel_subtest/string', String, queue_size=1)
    transform_pub = rospy.Publisher('/eel_subtest/transform', Transform, queue_size=1)
    pose_pub = rospy.Publisher('/eel_subtest/pose', Pose, queue_size=1)
    image_pub = rospy.Publisher('/eel_subtest/image', Image, queue_size=1)

    # 画像メッセージの作成
    img_msg = Image()
    img_msg.header.frame_id = "camera"
    img_msg.height = 480
    img_msg.width = 640
    img_msg.encoding = "rgb8"
    img_msg.is_bigendian = 0
    img_msg.step = 1920
    img_msg.data = [0] * (640 * 480 * 3)

    # Transformメッセージの作成
    transform_msg = Transform()
    transform_msg.translation.x = 1.0
    transform_msg.translation.y = 2.0
    transform_msg.translation.z = 3.0
    transform_msg.rotation.w = 1.0

    # Poseメッセージの作成
    pose_msg = Pose()
    pose_msg.position.x = 1.0
    pose_msg.position.y = 2.0
    pose_msg.position.z = 3.0
    pose_msg.orientation.w = 1.0

    # 一回だけパブリッシュして終了
    bool_pub.publish(Bool(True))
    int32_pub.publish(Int32(42))
    int64_pub.publish(Int64(9223372036854775807))
    float32_pub.publish(Float32(3.14159))
    float64_pub.publish(Float64(3.141592653589793))
    string_pub.publish(String("Hello ROS!"))
    transform_pub.publish(transform_msg)
    pose_pub.publish(pose_msg)
    image_pub.publish(img_msg)

    rate.sleep()
    rospy.signal_shutdown("Test complete")

if __name__ == '__main__':
    try:
        publish_test_data()
    except rospy.ROSInterruptException:
        pass