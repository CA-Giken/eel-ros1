# mypy: ignore-errors

import base64
from typing import List, TypeVar
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge

def publisher(topic_name: str, msg_class, **options):
    pub = rospy.Publisher(topic_name, msg_class, **options)
    return pub

def subscriber(topic_name: str, msg_class, callback):
    sub = rospy.Subscriber(topic_name, msg_class, callback)
    return sub

def get_msg_class(typ: str):
    """__summary__: 指定された型がimport可能なROS Messageであるかどうかを確認する
    Args:
        typ (str): ROS Messageの型
    """
    try:
        module_name, class_name = typ.rsplit('.', 1)
        module = __import__(module_name, fromlist=[class_name])
        msg_class = getattr(module, class_name)
    except Exception as e:
        print("[CA] Failed to import ROS Message class:", typ, e.args)
        return None
    return msg_class

def to_msg(typ: str, value: dict):
    """__summary__: ROS Message dataclass の動的インポートを行う
        例) typ = std_msgs.msg.String
        -> from std_msgs.msg import String
        -> String = String(data=value)

        例外) Image型はbase64stringからROS Messageに変換する
    """
    if typ == "sensor_msgs.msg.Image":
        return to_image_msg(value)

    try:
        module_name, class_name = typ.rsplit('.', 1)
        module = __import__(module_name, fromlist=[class_name])
        msg_class = getattr(module, class_name)
    except Exception as e:
        print("[CA] Failed to import ROS Message class:", typ, e.args)
        return None
    # ROS Message の作成
    try:
        msg = msg_class(**value)
    except Exception as e:
        print("[CA] Failed to create ROS Message:", typ, value, e.args)
        return None

    return msg

def from_msg(msg: TypeVar):
    """_summary_ ROS Message 型から辞書型に変換する
        例外)　Image型はbase64stringに変換する
    Args:
        msg (TypeVar): ROS Message

    Returns:
        dict: 辞書型
    """
    if isinstance(msg, Image):
        return from_image_msg(msg)

    def message_to_dict(msg):
        msg_dict = {}
        for slot in msg.__slots__:
            attr = getattr(msg, slot)
            if hasattr(attr, '__slots__'):
                msg_dict[slot] = message_to_dict(attr)
            else:
                msg_dict[slot] = attr
        return msg_dict

    msg_send = message_to_dict(msg)
    return msg_send


def to_image_msg(value: dict) -> Image:
    image = Image()
    image.height = value.height
    image.width = value.width
    image.encoding = value.encoding
    image.is_bigendian = value.is_bigendian
    image.step = value.step
    image.data = value.data
    return image

def from_image_msg(msg: Image) -> str:
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    _, buffer = cv2.imencode('.jpg', cv_image)
    base64string = base64.b64encode(buffer).decode("utf-8")
    return { base64string }

