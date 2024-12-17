# mypy: ignore-errors

import rospy
import eel
from std_msgs.msg import String

pubs = {} # { "/topic_name": { "publisher": rospy.Publisher, "last_value": value } }
subs = {} # { "/topic_name": { "subscriber": rospy.Subscriber, "last_value": value } }

# Subscribers
def callback_health(msg):
    value = eel.health(msg.data)
    print("[CA] Python -> JS: OK, ", value)

# rospy.Subscriber(f'/health', String, callback_health)
# Publishers