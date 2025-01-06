/**
 * デスクトップアプリ用途なのでEval警告を無視する
 */

globalThis.Utils = {
  EvalGetValue: function(value, key) {
    let v;
    eval("v = value" + key);
    return v;
  },
  EvalSetValue: function(value, key, newValue) {
    let v = structuredClone(value);
    eval("v" + key + " = newValue");
    return v;
  },
  /**
   * 辞書キーのフルパス指定して値追加
   * @param {*} dict 
   * @param {*} key: .options.data.meta[0] のようなフルパス
   * @param {*} value 
   * @returns dict
   */
  dictFullPathAdd: function(dict, key_fullpath, value) {
    let result = structuredClone(dict);
    let current = result;
    // 一番最初の.は無視
    if (key_fullpath.startsWith('.')) {
      key_fullpath = key_fullpath.slice(1);
    }

    // パスを分割して階層構造を作成
    const pathParts = key_fullpath.split('.');

    pathParts.forEach((part, index) => {
      // 配列のインデックスを処理
      const arrayMatch = part.match(/(\w+)\[(\d+)\]/);

      if (arrayMatch) {
        // 配列の場合
        const [_, arrayName, arrayIndex] = arrayMatch;
        if (!current[arrayName]) {
          current[arrayName] = [];
        }

        if (index === pathParts.length - 1) {
          // 最後の要素の場合は値を設定
          current[arrayName][arrayIndex] = value;
        } else {
          // 途中の要素の場合は空オブジェクトを作成
          if (!current[arrayName][arrayIndex]) {
            current[arrayName][arrayIndex] = {};
          }
          current = current[arrayName][arrayIndex];
        }
      } else {
        // 通常のオブジェクトの場合
        if (index === pathParts.length - 1) {
          // 最後の要素の場合は値を設定
          current[part] = value;
        } else {
          // 途中の要素の場合は空オブジェクトを作成
          if (!current[part]) {
            current[part] = {};
          }
          current = current[part];
        }
      }
    });

    return result;
  }
}