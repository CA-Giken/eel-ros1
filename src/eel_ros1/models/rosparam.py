import rospy
import time
import eel
import threading
GET_PARAM_INTERVAL = 5

PARAM_TYPES = {
    "Bool": "Bool",
    "Number": "Number",
    "String": "String",
    "Object": "Object",
    "List": "List"
}

rosparams = {} # { "/param_name": { "type": PARAM_TYPES, "value": value } }

getparam_loop_running = False
loop_thread = None

def getparam_loop():
    global getparam_loop_running, rosparams
    while getparam_loop_running:
        for key, value in rosparams.items():
            try:
                new_value = rospy.get_param(key)
                if new_value == value["value"]: # 値が変わっていない場合はスキップ
                    continue
                rosparams[key]["value"] = new_value

                # 値をUIに渡す
                eel.updateParam(key, new_value)
                print("[CA] Rosparam updated:", key, value["type"], new_value)
            except Exception as e:
                print("[CA] Failed to get rosparam:", key, e.args)

        # ５秒ごとに定期取得
        time.sleep(GET_PARAM_INTERVAL)

def run_getparam_loop():
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