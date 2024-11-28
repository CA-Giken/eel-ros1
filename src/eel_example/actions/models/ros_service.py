# mypy: ignore-errors

import rospy
import eel
import threading
import time

##### Parameterブロック

Config = {
    "package_name": "eel_example",
}

##### Functionブロック
pubs = {} # { "/topic_name": publisher }
subs = {} # { "/topic_name": subscriber }
rosparams = {} # { "/param_name": string | number | boolean }

getparam_loop_running = False
loop_thread = None

def getparam_loop():
    global getparam_loop_running, rosparams
    while getparam_loop_running:
        for key, value in rosparams.items():
            new_value = rospy.get_param(key)
            if new_value == value:
                continue
            rosparams[key] = new_value

            # 値をString型に変換してUIに渡す
            v = str(value)
            eel.updateParam(key, v)

        # ５秒ごとに定期取得
        time.sleep(5)

def run_gerparam_loop():
    global getparam_loop_running, loop_thread
    if not getparam_loop_running:
        getparam_loop_running = True
        loop_thread = threading.Thread(target=getparam_loop)
        loop_thread.start()
        print("[CA] Get rosparam loop has been started.")

def break_getparam_loop():
    global getparam_loop_running
    getparam_loop_running = False
    print("[CA] Get rosparam loop has been stopped.")

def register_param(param_name):
    global rosparams
    rosparams[param_name] = ""

def unregister_param(param_name):
    global rosparams
    rosparams.pop(param_name)

##### Mainブロック
rospy.init_node(Config["package_name"], anonymous = True)

try:
    Config.update(rospy.get_param("~config"))
except Exception as e:
    print("[CA] get_param exception:", e.args)

# Subscribers

# Publishers

# rospy.Timer(rospy.Duration(1), cb_scan, oneshot=True)