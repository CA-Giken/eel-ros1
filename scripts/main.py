#!/usr/bin/env python
# mypy: ignore-errors

import os
import eel
from jinja2 import Environment, FileSystemLoader
import rospkg

# Actionsをインポートして、このファイルにバンドルする
from eel_example.actions.actions import *  # noqa: F403
from eel_example.actions.models import ros_service

Config = ros_service.Config

# Jinja2の設定
rospack = rospkg.RosPack()
package_path = rospack.get_path(Config['package_name'])
abs_path = os.path.join(package_path, f'src/{Config["package_name"]}/templates')
os.makedirs(os.path.join(package_path, 'dist/web'), exist_ok=True)
env = Environment(loader=FileSystemLoader(abs_path))

# HTMLのレンダリング関数
def render_template(template_name, **context):
    template = env.get_template(template_name)
    rendered = template.render(**context)

    # 生成したHTMLをdist/web/フォルダに保存
    with open(os.path.join(package_path, f'dist/web/{template_name}'), 'w', encoding='utf-8') as f:
        f.write(rendered)

if __name__ == '__main__':
    # templatesフォルダ直下のHTMLファイルをdist/web/に保存
    files = os.listdir(abs_path)
    for file in files:
        if file.endswith('.html'):
            render_template(file)

    options = {
        "host": "0.0.0.0",
        "port": 8000,
        'cmdline_args': ["--no-sandbox"],
        'size': (800, 600)
    }
    dist_path = os.path.join(package_path, 'dist/web')
    print("Starting Eel app...")
    print("  dist path: ", dist_path)
    print("  hosted at:", f"http://localhost:{options['port']}")
    eel.init(dist_path)
    eel.start('index.html', **options)
    print("[CA] App quitted.")