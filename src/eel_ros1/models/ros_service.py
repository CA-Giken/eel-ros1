# mypy: ignore-errors

import rospy
import eel
from std_msgs.msg import String

def callback_health(msg):
    value = eel.health(msg.data)
    print("[CA] Python -> JS: OK, ", value)

class ROSService:
    """
    ROS Service Class

    Attributes:
        pubs (dict):
            {
                "/topic_name": {
                    "publisher": rospy.Publisher,
                    "last_value": ROS Message,
                    "cache": true/false
                }
            }
        subs (dict):
            {
                "/topic_name": {
                    "subscriber": rospy.Subscriber,
                    "last_value": ROS Message,
                    "cache": true/false
                }
            }
    """
    pubs = {}
    subs = {}

    @staticmethod
    def init(**kwargs):
        node_name = kwargs.get("node_name")

        rospy.Subscriber(f'{node_name}/health', String, ROSService.callback_health)
        print("[CA] ROS Service initialized.")

    @staticmethod
    def cache_sub(topic_name):
        # Subscribeしているトピックの最新値を、/cache/topic_nameにPublishする
        typ = ROSService.subs[topic_name]["subscriber"].data_class._type
        last_value = ROSService.subs[topic_name]["last_value"]
        pub = rospy.Publisher(f'/cache{topic_name}', typ, queue_size=1)
        pub.publish(last_value)