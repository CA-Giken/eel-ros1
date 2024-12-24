#!/usr/bin/env python
# mypy: ignore-errors

from eel_ros1.models.ros_wrapper import get_msg_class, to_msg
import rospy
from std_msgs.msg import Bool
from typing import Dict, Any

Config = {
    "cache": []
}
Param = {}


#### Functionブロック
cache_topics: Dict[str, Dict[str, Any]] = {}

def reload_cache(bool_msg: Bool):
    for topic in cache_topics:
        cache_topics[topic]["publisher"].publish(cache_topics[topic]["value"])

def cache_callback(msg, topic_to):
    # 最終値をキャッシュ
    cache_topics[topic_to]["value"] = msg
    pub = cache_topics[topic_to].publisher
    pub.publish(msg)

#### Mainブロック
rospy.init_node("cache", anonymous=True)
try:
    Config.update(rospy.get_param("~config"))
except Exception as e:
    print("get_param exception:", e.args)

#### Subscribers
rospy.Subscriber("/cache/reload", Bool, reload_cache)

#### Cache Topics
for topic_config in Config["cache"]:
    # Config Validation
    topic_name = topic_config.get("topic", None)
    typ = topic_config.get("m-type", None)
    topic_to = topic_config.get("to", None)
    topic_file = topic_config.get("file", None)

    assert typ is not None, f"Message type {typ} not found"
    dataclass = get_msg_class(typ)

    if topic_name:
        mode = "topic"
        assert topic_file is None, "'file' must not be specified on Topic cache mode."
        if topic_to is None:
            topic_to = f"/cache{topic_name}"
    else:
        # topic がない場合は、ファイルインポートモード
        mode = "file"
        assert topic_file is not None, "'topic' or 'file' must be specified"
        assert topic_to is not None, "'to' must be specified on File cache mode."
        # ファイル読み込みは現在画像ファイルのみをサポート, jpg or png
        assert topic_file.endswith(".jpg") or topic_file.endswith(".png"), "Only jpg or png file is supported"

    assert topic_to.startswith("/cache/"), f"Invalid topic_to: {topic_to}"

    # Caching
    pub = rospy.Publisher(topic_name, typ, queue_size=10)
    cache_topics[topic_to] = {
        "publisher": pub,
        "value": None
    }
    if mode == "topic":
        sub = rospy.Subscriber(topic_name, typ, cache_callback, callback_args=topic_to)
        cache_topics[topic_to]["subscriber"] = sub
    if mode == "file":
        with open(topic_file, "r") as f:
            data = f.read()
            msg = to_msg(typ, data)
            cache_topics[topic_to]["value"] = msg