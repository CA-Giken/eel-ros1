#!/usr/bin/env python
# -*- coding: utf-8 -*-
# mypy: ignore-errors

import eel
from jinja2 import Environment, FileSystemLoader
from handlers.example import *  # noqa: F403

# Jinja2の設定
env = Environment(loader=FileSystemLoader('templates'))

# HTMLのレンダリング関数
def render_template(template_name, **context):
    template = env.get_template(template_name)
    rendered = template.render(**context)

    # 生成したHTMLをdist/web/フォルダに保存
    with open(f'dist/web/{template_name}', 'w', encoding='utf-8') as f:
        f.write(rendered)

if __name__ == '__main__':
    render_template('index.html', title='ランダム数字ジェネレーター')

    eel.init('dist/web')
    eel.start('index.html', size=(800, 600))