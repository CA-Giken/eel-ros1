# mypy: ignore-errors

import rospy
import eel
from std_msgs.msg import String

##### Parameterブロック
Config = {
    "package_name": "eel_ros1",
    "log_level": "info", # debug | info | warn | error
}

##### Functionブロック
pubs = {} # { "/topic_name": { "publisher": rospy.Publisher, "last_value": value } }
subs = {} # { "/topic_name": { "subscriber": rospy.Subscriber, "last_value": value } }

##### Mainブロック
rospy.init_node(Config["package_name"], anonymous = True)

try:
    Config.update(rospy.get_param("~config"))
except Exception as e:
    print("[CA] get_param exception:", e.args)

# Subscribers
def callback_health(msg):
    value = eel.health(msg.data)
    print("[CA] Python -> JS: OK, ", value)

rospy.Subscriber(f'{Config["package_name"]}/health', String, callback_health)
# Publishers

# rospy.Timer(rospy.Duration(1), cb_scan, oneshot=True)