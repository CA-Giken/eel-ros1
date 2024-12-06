#!/usr/bin/env python

import os
from jinja2 import Environment, FileSystemLoader

package_path: str
template_path: str
dist_path: str
env: Environment


# HTMLのレンダリング関数
def render_template(filepath, **context):
    template = env.get_template(filepath)
    rendered = template.render(**context)

    # 生成したHTMLをdist/web/フォルダに保存
    with open(f'{dist_path}/{filepath}', 'w', encoding='utf-8') as f:
        f.write(rendered)

def copy_js(filepath):
    with open(os.path.join(template_path, filepath), 'r', encoding='utf-8') as f:
        js = f.read()
        with open(f'{dist_path}/{filepath}', 'w', encoding='utf-8') as f:
            f.write(js)

def copy_css(filepath):
    with open(os.path.join(template_path, filepath), 'r', encoding='utf-8') as f:
        css = f.read()
        with open(f'{dist_path}/{filepath}', 'w', encoding='utf-8') as f:
            f.write(css)

def convert_csv_to_json(filepath):
    import csv
    import json
    with open(os.path.join(template_path, filepath), 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        descriptions = list(reader)
    # dist/web ディレクトリに出力
    with open(f'{dist_path}/{filepath.replace(".csv", ".json")}', 'w', encoding='utf-8') as f:
        json.dump(descriptions, f, ensure_ascii=False, indent=2)

def bundle(root, template = "templates", dist = "dist/web"):
    # templatesフォルダ直下のHTMLファイルをdist/web/に保存
    global package_path, template_path, dist_path, env
    package_path = root
    template_path = os.path.join(package_path, template)
    dist_path = os.path.join(package_path, dist)
    os.makedirs(dist_path, exist_ok=True)
    env = Environment(loader=FileSystemLoader(template_path))

    files = os.listdir(template_path)
    for file in files:
        if file.endswith('.html'):
            render_template(file)
        if file.endswith('.js'):
            copy_js(file)
        if file.endswith('.css'):
            copy_css(file)
        if file.endswith('.csv'):
            convert_csv_to_json(file)

    return dist_path
