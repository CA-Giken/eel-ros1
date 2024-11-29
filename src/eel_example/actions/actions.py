# mypy: ignore-errors
import eel
import rospy

from eel_example.actions.models.ros_wrapper import to_msg, from_msg, MSG_TYPES, publisher, subscriber
from eel_example.actions.models.ros_service import pubs, subs

@eel.expose
def health(value):
    print("[CA] JS->Python: OK")
    return value

@eel.expose
def ros_publish(topic_name: str, type: str, value: str):
    assert type in MSG_TYPES.keys(), f"[CA] Invalid message type: {type}. Available types: {MSG_TYPES.values()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    if topic_name in pubs.keys():
        pub = pubs[topic_name]
    else:
        pub = publisher(topic_name, MSG_TYPES[type], queue_size=1)
        pubs[topic_name] = pub

    msg = to_msg(MSG_TYPES[type], value)
    pub.publish(msg)
    print("[CA] Published: ", topic_name, MSG_TYPES[type], value)
    # TODO: PublisherのLifetimeをどうするか？

@eel.expose
def ros_set_param(param_name: str, param_value):
    rospy.set_param(param_name, param_value)
    print("[CA] Rosparam set: ", param_name, param_value)

@eel.expose
def ros_subscribe(topic_name: str, type: str):
    print(topic_name, type)
    assert type in MSG_TYPES.keys(), f"[CA] Invalid message type: {type}. Available types: {MSG_TYPES.keys()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    def callback(value):
        print("[CA]", topic_name, MSG_TYPES[type], value)
        eel.updateSubscribedValue(topic_name, MSG_TYPES[type], from_msg(MSG_TYPES[type], value))

    if topic_name in subs.keys():
        sub = subs[topic_name]
    else:
        sub = subscriber(topic_name, MSG_TYPES[type], callback)
        subs[topic_name] = sub
        print("[CA] Subscriber: ", topic_name, MSG_TYPES[type])
