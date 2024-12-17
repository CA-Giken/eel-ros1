#!/usr/bin/env python
# mypy: ignore-errors

import os
from jinja2 import Environment, FileSystemLoader
import sys

from utils import generate_id

js_packages = [
    {
        "url": "https://cdn.jsdelivr.net/npm/zod@3.22.4/lib/index.umd.min.js",
        "path": 'zod.js'
    }
]

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
        save_path = f'{dist_path}/{filepath.replace(".mock", "")}' # モックファイルは.mockを取り除く
        with open(save_path, 'w', encoding='utf-8') as f:
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

    # jinja2内で使う関数を登録
    env.globals.update(generate_id=generate_id)

    files = os.listdir(template_path)
    for file in files:
        if file.endswith('.html'):
            render_template(file)
        # Viteで直接dist/webにバンドルするので、ファイルは不要
        # if file.endswith('bundle.umd.js'):
        #     copy_js(file)
        if file.endswith('.mock.js'): # パッケージのモックファイルはそのままコピー
            copy_js(file)
        if file.endswith('.css'):
            copy_css(file)
        if file.endswith('.csv'):
            convert_csv_to_json(file)

    return dist_path

def download_js_files(dist_path):
    import requests
    # JSパッケージのインストール
    # パッケージ群は、`dist/web/static`にインストールされる。
    # 既にインストールされている場合はバージョン確認はせず、スキップされる
    static_path = os.path.join(dist_path, 'static')
    os.makedirs(static_path, exist_ok=True)

    for package in js_packages:
        try:
            response = requests.get(package["url"])
            response.raise_for_status()

            package_path = os.path.join(static_path, package["path"])
            with open(package_path, 'w', encoding='utf-8') as f:
                f.write(response.text)
            print(f"Zod successfully downloaded to {package['path']}")
        except Exception as e:
            print(f"Error downloading Zod: {e}")

# ROSパッケージ直下に配置されたシンボリックリンクから呼び出される事を想定
if __name__ == '__main__':
    dist_path = bundle(os.path.dirname(os.path.abspath(sys.argv[0])))
    download_js_files(dist_path)

