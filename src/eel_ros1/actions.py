# mypy: ignore-errors
import eel
import rospy

from eel_ros1.models.ros_wrapper import to_msg, from_msg, MSG_TYPES, publisher, subscriber
from eel_ros1.models.ros_service import pubs, subs, Config
from eel_ros1.models.rosparam import PARAM_TYPES, rosparams

@eel.expose
def health(value):
    print("[CA] JS->Python: OK", value)
    return value

@eel.expose
def ros_publish(topic_name: str, typ: str, value: str):
    assert typ in MSG_TYPES.values(), f"[CA] Invalid message type: {typ}. Available types: {MSG_TYPES.values()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    if topic_name in pubs.keys():
        pub = pubs[topic_name]
    else:
        pubs[topic_name] = {
            "publisher": publisher(topic_name, typ, queue_size=1),
            "last_value": None
        }
        pub = pubs[topic_name]

    msg = to_msg(typ, value)
    pub["publisher"].publish(msg)
    pub["last_value"] = value
    print("[CA] Published:", topic_name, typ, value)
    # TODO: PublisherのLifetimeをどうするか？

@eel.expose
def ros_subscribe(topic_name: str, typ: str):
    assert typ in MSG_TYPES.values(), f"[CA] Invalid message type: {typ}. Available types: {MSG_TYPES.values()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    def callback(value):
        if Config["log_level"] == "debug":
            print("[CA]", topic_name, typ, value)
        eel.updateSubscribedValue(topic_name, typ, from_msg(typ, value))
        subs[topic_name]["last_value"] = value
    if topic_name in subs.keys():
        sub = subs[topic_name]
    else:
        subs[topic_name] = {
            "subscriber": subscriber(topic_name, typ, callback),
            "last_value": None
        }
        sub = subs[topic_name]
        print("[CA] Subscriber:", topic_name, typ)

@eel.expose
def ros_unsubscribe(topic_name: str):
    assert topic_name in subs.keys(), f"[CA] Invalid topic name: {topic_name}. Not subscribed yet."

    try:
        subs[topic_name]["subscriber"].unregister()
        subs.pop(topic_name)
        print("[CA] Unsubscribed:", topic_name)
    except Exception as e:
        print("[CA] Failed to unsubscribe:", topic_name, e.args)

@eel.expose
def ros_set_param(param_name: str, typ: str, param_value):
    assert typ in PARAM_TYPES.values(), f"[CA] Invalid ROSParam type: {typ}. Available types: {PARAM_TYPES.values()}"
    assert rosparams[param_name]["type"] == typ, f"[CA] Invalid ROSParam type: {typ}. Expected: {rosparams[param_name]['type']}"

    try:
        rospy.set_param(param_name, param_value)
        rosparams[param_name]["value"] = param_value
        print("[CA] Rosparam set:", param_name, param_value)
    except Exception as e:
        print("[CA] Failed to set rosparam:", param_name, param_value, e.args)

@eel.expose
def ros_register_param(param_name: str, typ: str):
    assert typ in PARAM_TYPES.values(), f"[CA] Invalid ROSParam type: {typ}. Available types: {PARAM_TYPES.values()}"

    try:
        value = rospy.get_param(param_name)
        rosparams[param_name] = {
            "type": typ,
            "value": value
        }
        eel.updateParam(param_name, typ, value)
        print("[CA] Rosparam registered:", param_name, value)
    except Exception as e:
        print("[CA] Failed to register rosparam:", param_name, e.args)

@eel.expose
def ros_unregister_param(param_name: str):
    rosparams.pop(param_name)
    print("[CA] Rosparam unregistered:", param_name)