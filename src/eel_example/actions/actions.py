# mypy: ignore-errors
import eel
import rospy

from eel_example.actions.models.ros_wrapper import to_msg, MSG_TYPES, publisher, subscriber
from eel_example.actions.models.ros_service import pubs, subs

@eel.expose
def ros_publish(topic_name: str, type: str, value: str):
    assert type in MSG_TYPES.values(), f"[CA] Invalid message type: {type}. Available types: {MSG_TYPES.values()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    if topic_name in pubs:
        pub = pubs[topic_name]
    else:
        pub = publisher(topic_name, type, queue_size=1)
        pubs[topic_name] = pub

    msg = to_msg(type, value)
    pub.publish(msg)

    # TODO: PublisherのLifetimeをどうするか？

@eel.expose
def ros_set_param(param_name: str, param_value):
    rospy.set_param(param_name, param_value)

@eel.expose
def ros_subscribe(topic_name: str, type: str):
    assert type in MSG_TYPES.values(), f"[CA] Invalid message type: {type}. Available types: {MSG_TYPES.values()}"
    assert topic_name.startswith('/'), f"[CA] Invalid topic name: {topic_name}. Must starts with '/"

    def callback(value):
        eel.updateSubscribedValue(topic_name, type, value)

    if topic_name in subs:
        sub = subs[topic_name]
    else:
        sub = subscriber(topic_name, type, callback)
        subs[topic_name] = sub
