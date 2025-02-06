# mypy: ignore-errors
import eel
import rospy

from eel_ros1.models.ros_wrapper import to_msg, from_msg, publisher, subscriber, get_msg_class
from eel_ros1.models.ros_service import ROSService
from eel_ros1.models.rosparam import PARAM_TYPES, rosparams

@eel.expose
def health(value):
    print("[CA] JS->Python: OK", value)
    return value

@eel.expose
def ros_publish(topic_name: str, typ: str, value: str, cache: bool = False):
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/'"

    msg_class = get_msg_class(typ)

    if topic_name in ROSService.pubs.keys():
        pub = ROSService.pubs[topic_name]
    else:
        ROSService.pubs[topic_name] = {
            "publisher": publisher(topic_name, msg_class, queue_size=1),
            "last_value": None,
            "cache": cache
        }
        pub = ROSService.pubs[topic_name]

    msg = to_msg(typ, value)
    pub["publisher"].publish(msg)
    pub["last_value"] = msg
    print("[CA] Published:", topic_name, msg)
    # TODO: PublisherのLifetimeをどうするか？

@eel.expose
def ros_subscribe(topic_name: str, typ: str, cache: bool = False):
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/'"
    msg_class = get_msg_class(typ)
    def callback(msg):
        eel.updateSubscribedValue(topic_name, from_msg(msg))
        ROSService.subs[topic_name]["last_value"] = msg
        if ROSService.subs[topic_name]["cache"]:
            ROSService.cache_sub(topic_name)

    if topic_name in ROSService.subs.keys():
        sub = ROSService.subs[topic_name]
    else:
        ROSService.subs[topic_name] = {
            "subscriber": subscriber(topic_name, msg_class, callback),
            "last_value": None,
            "cache": cache
        }
        sub = ROSService.subs[topic_name]
        print("[CA] Subscriber:", topic_name, msg_class)

@eel.expose
def ros_unsubscribe(topic_name: str):
    assert topic_name in ROSService.subs.keys(), f"[CA] Invalid topic name: {topic_name}. Not subscribed yet."

    try:
        ROSService.subs[topic_name]["subscriber"].unregister()
        ROSService.subs.pop(topic_name)
        print("[CA] Unsubscribed:", topic_name)
    except Exception as e:
        print("[CA] Failed to unsubscribe:", topic_name, e.args)

@eel.expose
def ros_set_param(param_name: str, typ: str, param_value):
    global rosparams
    assert typ in PARAM_TYPES.values(), f"[CA] Invalid ROSParam type: {typ}. Available types: {PARAM_TYPES.values()}"
    assert rosparams[param_name]["type"] == typ, f"[CA] Invalid ROSParam type: {typ}. Expected: {rosparams[param_name]['type']}"

    try:
        rospy.set_param(param_name, param_value)
        rosparams[param_name]["value"] = param_value
        print("[CA] Rosparam set:", param_name, param_value)
        eel.updateParam(param_name, typ, param_value)
    except Exception as e:
        print("[CA] Failed to set rosparam:", param_name, param_value, e.args)

@eel.expose
def ros_register_param(param_name: str):
    global rosparams

    rosparams[param_name] = {
        "type": None,
        "value": None
    }
    try:
        value = rospy.get_param(param_name)
        rosparams[param_name]["value"] = value

        # Get the type of the parameter
        if isinstance(value, bool):
            rosparams[param_name]["type"] = PARAM_TYPES["Bool"]
        elif isinstance(value, int):
            rosparams[param_name]["type"] = PARAM_TYPES["Number"]
        elif isinstance(value, float):
            rosparams[param_name]["type"] = PARAM_TYPES["Number"]
        elif isinstance(value, str):
            rosparams[param_name]["type"] = PARAM_TYPES["String"]
        elif isinstance(value, list):
            rosparams[param_name]["type"] = PARAM_TYPES["List"]
        elif isinstance(value, dict):
            rosparams[param_name]["type"] = PARAM_TYPES["Object"]
        else:
            raise TypeError(f"Invalid type: {type(value)}")
        print("[CA] Rosparam registered:", param_name, value)
    except Exception as e:
        print("[CA] Failed to register rosparam:", param_name, e.args)

    eel.updateParam(
        param_name,
        rosparams[param_name]["value"]
    )

@eel.expose
def ros_unregister_param(param_name: str):
    rosparams.pop(param_name)
    print("[CA] Rosparam unregistered:", param_name)

@eel.expose
def ros_get_param(param_name: str):
    assert param_name in rosparams.keys(), f"[CA] Invalid ROSParam name: {param_name}. Not registered yet."

    return rosparams[param_name]["value"]