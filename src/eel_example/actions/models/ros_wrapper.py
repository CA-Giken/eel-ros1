# mypy: ignore-errors

import base64
from typing import List, TypeVar
from geometry_msgs.msg import Transform, Pose
from std_msgs.msg import Bool, Int32, Int64, Float32, Float64, String
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge

MSG_TYPES = {
    "Bool": 'Bool:std_msgs',
    "Int32": 'Int32:std_msgs',
    "Int64": 'Int64:std_msgs',
    "Float32": 'Float32:std_msgs',
    "Float64": 'Float64:std_msgs',
    "String": 'String:std_msgs',
    "Transform": 'Transform:geometry_msgs',
    "Pose": 'Pose:geometry_msgs',
    "Image": 'Image:sensor_mags'
}
def publisher(topic_name: str, type: str, **options):
    if type == MSG_TYPES["Bool"]:
        pub = rospy.Publisher(topic_name, Bool, **options)
    elif type == MSG_TYPES["Int32"]:
        pub = rospy.Publisher(topic_name, Int32, **options)
    elif type == MSG_TYPES["Int64"]:
        pub = rospy.Publisher(topic_name, Int64, **options)
    elif type == MSG_TYPES["Float32"]:
        pub = rospy.Publisher(topic_name, Float32, **options)
    elif type == MSG_TYPES["Float64"]:
        pub = rospy.Publisher(topic_name, Float64, **options)
    elif type == MSG_TYPES["String"]:
        pub = rospy.Publisher(topic_name, String, **options)
    elif type == MSG_TYPES["Transform"]:
        pub = rospy.Publisher(topic_name, Transform, **options)
    elif type == MSG_TYPES["Pose"]:
        pub = rospy.Publisher(topic_name, Pose, **options)
    elif type == MSG_TYPES["Image"]:
        pub = rospy.Publisher(topic_name, Image, **options)
    else:
        raise ValueError("[CA] Unexpected ROS Message type: ", type)
    return pub

def subscriber(topic_name: str, type: MSG_TYPES, callback):
    if type == MSG_TYPES["Bool"]:
        sub = rospy.Subscriber(topic_name, Bool, callback)
    elif type == MSG_TYPES["Int32"]:
        sub = rospy.Subscriber(topic_name, Int32, callback)
    elif type == MSG_TYPES["Int64"]:
        sub = rospy.Subscriber(topic_name, Int64, callback)
    elif type == MSG_TYPES["Float32"]:
        sub = rospy.Subscriber(topic_name, Float32, callback)
    elif type == MSG_TYPES["Float64"]:
        sub = rospy.Subscriber(topic_name, Float64, callback)
    elif type == MSG_TYPES["String"]:
        sub = rospy.Subscriber(topic_name, String, callback)
    elif type == MSG_TYPES["Transform"]:
        sub = rospy.Subscriber(topic_name, Transform, callback)
    elif type == MSG_TYPES["Pose"]:
        sub = rospy.Subscriber(topic_name, Pose, callback)
    elif type == MSG_TYPES["Image"]:
        sub = rospy.Subscriber(topic_name, Image, callback)
    else:
        raise ValueError("[CA] Unexpected ROS Message type: ", type)
    return sub

def to_msg(type: str, value: str):
    if type == MSG_TYPES["Bool"]:
        msg = to_bool_msg(value)
    elif type == MSG_TYPES["Int32"]:
        msg = to_int32_msg(value)
    elif type == MSG_TYPES["Int64"]:
        msg = to_int64_msg(value)
    elif type == MSG_TYPES["Float32"]:
        msg = to_float32_msg(value)
    elif type == MSG_TYPES["Float64"]:
        msg = to_float64_msg(value)
    elif type == MSG_TYPES["String"]:
        msg = to_string_msg(value)
    elif type == MSG_TYPES["Transform"]:
        msg = to_transform_msg(value)
    elif type == MSG_TYPES["Pose"]:
        msg = to_pose_msg(value)
    elif type == MSG_TYPES["Image"]:
        msg = to_image_msg(value)

    return msg

def from_msg(type: str, msg: str):
    if type == MSG_TYPES["Bool"]:
        value = from_bool_msg(msg)
    elif type == MSG_TYPES["Int32"]:
        value = from_int32_msg(msg)
    elif type == MSG_TYPES["Int64"]:
        value = from_int64_msg(msg)
    elif type == MSG_TYPES["Float32"]:
        value = from_float32_msg(msg)
    elif type == MSG_TYPES["Float64"]:
        value = from_float64_msg(msg)
    elif type == MSG_TYPES["String"]:
        value = from_string_msg(msg)
    elif type == MSG_TYPES["Transform"]:
        value = from_transform_msg(msg)
    elif type == MSG_TYPES["Pose"]:
        value = from_pose_msg(msg)
    elif type == MSG_TYPES["Image"]:
        value = from_image_msg(msg)

    return value

def to_bool_msg(value: str) -> Bool:
    if value == 'True':
        return True
    elif value == 'False':
        return False
    else:
        raise ValueError(f"[CA] Invalid value: {value}. Must be 'true' or 'false'.")

def to_int32_msg(value: str) -> Int32:
    assert value.isdigit(), f"[CA] Invalid value: {value}. Must be a digit."
    assert -2**31 <= int(value) <= 2**31 - 1, f"[CA] Invalid value: {value}. Must be in the range of int32."
    assert int(value) == float(value), f"[CA] Invalid value: {value}. Must be an integer."
    return int(value)

def to_int64_msg(value: str) -> Int64:
    assert value.isdigit(), f"[CA] Invalid value: {value}. Must be a digit."
    assert -2**63 <= int(value) <= 2**63 - 1, f"[CA] Invalid value: {value}. Must be in the range of int64."
    assert int(value) == float(value), f"[CA] Invalid value: {value}. Must be an integer."
    return int(value)

def to_float32_msg(value: str) -> Float32:
    assert value.replace('.', '', 1).isdigit(), f"[CA] Invalid value: {value}. Must be a digit."
    assert -3.4e38 <= float(value) <= 3.4e38, f"[CA] Invalid value: {value}. Must be in the range of float32."
    return float(value)

def to_float64_msg(value: str) -> Float64:
    assert value.replace('.', '', 1).isdigit(), f"[CA] Invalid value: {value}. Must be a digit."
    assert -1.7e308 <= float(value) <= 1.7e308, f"[CA] Invalid value: {value}. Must be in the range of float64."
    return float(value)

def to_string_msg(value: str) -> String:
    return value

def to_transform_msg(value: str) -> Transform:
    # Expect value to be a string like "x y z qx qy qz qw"
    values = value.split()
    assert len(values) == 7, f"[CA] Invalid value: {value}. Must be 'x y z qx qy qz qw'."
    assert all(val.replace('.', '', 1).isdigit() for val in values), f"[CA] Invalid value: {value}. Must be a digit."
    return values

def to_pose_msg(value: str) -> Pose:
    # Expect value to be a string like "x y z qx qy qz qw"
    values = value.split()
    assert len(values) == 7, f"[CA] Invalid value: {value}. Must be 'x y z qx qy qz qw'."
    assert all(val.replace('.', '', 1).isdigit() for val in values), f"[CA] Invalid value: {value}. Must be a digit."
    return values

def to_image_msg(value: str) -> Image:
    # Expect value to be a string like "width height encoding is_bigendian step data"
    values = value.split()
    assert len(values) == 6, f"[CA] Invalid value: {value}. Must be 'width height encoding is_bigendian step data'."
    assert all(val.isdigit() for val in values[:4]), f"[CA] Invalid value: {value}. Must be a digit."
    return values

def from_bool_msg(msg: Bool) -> str:
    return str(msg.data)

def from_int32_msg(msg: Int32) -> str:
    return str(msg.data)

def from_int64_msg(msg: Int64) -> str:
    return str(msg.data)

def from_float32_msg(msg: Float32) -> str:
    return str(msg.data)

def from_float64_msg(msg: Float64) -> str:
    return str(msg.data)

def from_string_msg(msg: String) -> str:
    return str(msg.data)

def from_transform_msg(msg: Transform) -> List[str]:
    values: List[str] = []
    values.append(msg.translation.x)
    values.append(msg.translation.y)
    values.append(msg.translation.z)
    values.append(msg.rotation.x)
    values.append(msg.rotation.y)
    values.append(msg.rotation.z)
    values.append(msg.rotation.w)

def from_pose_msg(msg: Transform) -> List[str]:
    values: List[str] = []
    values.append(msg.position.x)
    values.append(msg.position.y)
    values.append(msg.position.z)
    values.append(msg.orientation.x)
    values.append(msg.orientation.y)
    values.append(msg.orientation.z)
    values.append(msg.orientation.w)

def from_image_msg(msg: Image) -> str:
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    _, buffer = cv2.imencode('.jpg', cv_image)
    base64string = base64.b64encode(buffer).decode("utf-8")
    return base64string