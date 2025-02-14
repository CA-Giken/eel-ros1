#
# Jinja2のテンプレートタグ内で使用する関数群
#
from markupsafe import Markup, escape


def generate_id():
    """UUIDを生成する

    Returns:
        _type_: _description_
    """
    import uuid
    return str(uuid.uuid4())

def replace_attributes(options = {}):
    """予約済属性を削除する

    予約済属性: class, style, name, value, map, item, on-*
    除外属性: type

    Args:
        attrs (dict): 属性の辞書

    Returns:
        dict: 予約済属性を削除した属性の辞書
    """
    attrs = options.copy()

    # 予約済属性を削除
    for key in list(attrs.keys()):
        if key in ['class', 'style', 'type', 'name', 'value', 'map', 'item']:
            del attrs[key]
        if key.startswith('on-'):
            del attrs[key]

    return attrs

def spread_attributes(options={}):
    """
    HTML属性を展開する
    """
    return Markup(' '.join([f'{key}="{value}"' for key, value in options.items()]))

def include_code(filename, env):
    """
    ファイルの内容を読み込み、HTMLエスケープしてpre/codeタグで囲む
    """
    try:
        # FileSystemLoaderのget_sourceを使用してファイルを読み込む
        source, _, _ = env.loader.get_source(env, filename)
        # HTMLをエスケープ
        escaped_content = escape(source)
        return f'<pre><code>{escaped_content}</code></pre>'
    except Exception as e:
        print(e)
        return f'[CA] Error reading {filename}: {str(e)}'