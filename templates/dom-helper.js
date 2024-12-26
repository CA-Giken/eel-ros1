class DOMUpdateHelper {
  static updateCallbacks = new WeakMap();

  /**
   * DOM更新コールバックの登録
   * @param {*} element
   * @param {*} callback: ({ element, ...payload }) => { ... }
   */
  static registerCallback(element, callback) {
    if (DOMUpdateHelper.updateCallbacks.has(element)) {
      DOMUpdateHelper.updateCallbacks.get(element).push(callback);
    } else {
      DOMUpdateHelper.updateCallbacks.set(element, [callback]);
    }
  }

  /**
   * DOM更新コールバックの実行
   * @param {*} element
   * @param {*} payload
   */
  static executeCallbacks(element, payload) {
    if (DOMUpdateHelper.updateCallbacks.has(element)) {
      DOMUpdateHelper.updateCallbacks.get(element).forEach((callback) => {
        callback({ element, ...payload });
      });
    }
  }

  /**
   * DOM更新コールバック
   * @param {*} payload
   */
  static updateElement(payload) {
    const { element, value } = payload;
    switch (element.tagName) {
      case "DIV":
        DOMUpdateHelper.updateDivElement(element, value);
        break;
      case "SELECT":
        DOMUpdateHelper.updateSelectElement(element, value);
        break;
      case "INPUT":
        DOMUpdateHelper.updateInputElement(element, value);
        break;
      case "IMG":
        DOMUpdateHelper.updateImageElement(element, value);
        break;
      default:
        throw new Error(`Unsupported element tag: ${element.tagName}`);
    }
  }

  static updateDivElement(element, value) {
    element.setAttribute("data-value", value);

    const children = element.children;
    for (const child of children) {
      // ext属性を持つ子要素のみ再帰探索
      if (!child.hasAttribute("ext")) continue;
      DOMUpdateHelper.updateElement({
        element: child,
        value: Utils.EvalGetValue(value, child.getAttribute("ext"))
      });
    }
  }

  static updateInputElement(element, value) {
    if (element.type === "checkbox") {
      element.checked = value;
      return;
    }
    if (element.type === "radio") {
      element.checked = element.value === value;
      return;
    }
    element.value = value;
    return;
  }

  static updateSelectElement(element, value) {
    const options = element.querySelectorAll("option");
    for (let i = 0; i < options.length; i++) {
      options[i].selected = options[i].value === value;
    }
  }

  static updateImageElement(element, value) {
    element.src = value;
  }
}
globalThis.domUpdateHelper = DOMUpdateHelper;