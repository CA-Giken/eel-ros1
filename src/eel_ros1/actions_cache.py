import eel
import rospy
from std_msgs.msg import Bool

from eel_ros1.models.ros_wrapper import dataclass_to_msg_type, from_msg
from eel_ros1.models.ros_service import ROSService

WAIT_FOR_MESSAGE_TIMEOUT = 1

eel.expose()
def reload_cache():
    # ROSService.subsのうち、cache=Trueのsubscriberに対して、/cache/{topic_name} を一度だけSubscribeする
    for topic_name, sub in ROSService.subs.items():
        if sub["cache"]:
            dataclass = sub["subscriber"].data_class
            typ = dataclass_to_msg_type(dataclass)

            def callback(value):
                eel.updateSubscribedValue(topic_name, typ, from_msg(typ, value))

            rospy.WaitForMessage(f"/cache/{topic_name}", dataclass, timeout=WAIT_FOR_MESSAGE_TIMEOUT, callback=callback)

    publisher = rospy.Publisher("/cache/reload", Bool, queue_size=1)
    publisher.publish(Bool(True))