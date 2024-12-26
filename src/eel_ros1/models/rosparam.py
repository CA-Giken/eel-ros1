import rospy
import eel

GET_PARAM_INTERVAL = 5

PARAM_TYPES = {
    "Bool": "Bool",
    "Number": "Number",
    "String": "String",
    "Object": "Object",
    "List": "List"
}

rosparams = {} # { "/param_name": { "type": PARAM_TYPES, "value": value } }

def getparam_loop():
    while True:
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
        eel.sleep(GET_PARAM_INTERVAL)