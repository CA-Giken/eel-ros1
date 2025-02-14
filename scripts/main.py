#!/usr/bin/env python
# mypy: ignore-errors

import os
import sys
import eel
import rospkg
import argparse
import rospy

# Actionsをインポートして、このファイルにバンドルする
from eel_ros1.actions import *  # noqa: F403
from eel_ros1.models import rosparam # FIXME: おそらくros_serviceのインポートはここ必須

PACKAGE_NAME = "eel-ros1"
OPTIONS = {
    "host": "0.0.0.0",
    "port": 8000,
    'cmdline_args': ["--no-sandbox"],
    'size': (800, 600),
    "block": True,
    "mode": "chrome"
}

# Jinja2の設定
rospack = rospkg.RosPack()
package_path = rospack.get_path(PACKAGE_NAME)
abs_path = os.path.join(package_path, 'templates')

# コマンドライン引数の処理
argv = rospy.myargv(argv=sys.argv)
parser = argparse.ArgumentParser(description="EEL Example")
parser.add_argument("--html_dir", help="HTML directory path")
parser.add_argument("--port", help="Port number")
args = parser.parse_args(argv[1:])
if args.html_dir:
    abs_path = args.html_dir
if args.port:
    OPTIONS['port'] = args.port

##### Mainブロック
rospy.init_node(PACKAGE_NAME, anonymous = True)
dist_path = os.path.join(package_path, 'dist/web')
print("Starting Eel app...")
print("  dist path: ", dist_path)
print("  hosted at:", f"http://{OPTIONS['host']}:{OPTIONS['port']}")
eel.init(dist_path)
eel.spawn(rosparam.getparam_loop)
eel.start('index.html', **OPTIONS)

print("Eel app stopped.")