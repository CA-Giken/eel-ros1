class DOMUpdateHelper {
  constructor() {
    this.updateCallbacks = new WeakMap();
  }

  /**
   * DOM更新コールバックの登録
   * @param {*} element
   * @param {*} callback: ({ element, ...payload }) => { ... }
   */
  registerCallback(element, callback) {
    if (this.updateCallbacks.has(element)) {
      this.updateCallbacks.get(element).push(callback);
    } else {
      this.updateCallbacks.set(element, [callback]);
    }
  }

  /**
   * DOM更新コールバックの実行
   * @param {*} element
   * @param {*} payload
   */
  executeCallbacks(element, payload) {
    if (this.updateCallbacks.has(element)) {
      this.updateCallbacks.get(element).forEach((callback) => {
        callback({ element, ...payload });
      });
    }
  }

  /**
   * DOM更新コールバック
   * @param {*} element 
   * @param {*} value 
   */
  updateElement(element, value) {
    switch (element.tagName) {
      case "DIV":
        updateDivElement(element, value);
        break;
      case "SELECT":
        updateSelectElement(element, value);
        break;
      case "INPUT":
        updateInputElement(element, value);
        break;
      case "IMG":
        updateImageElement(element, value);
      default:
        throw new Error(`Unsupported element type: ${element.tagName}`);
    }
  }

  updateDivElement(element, value) {
    element.setAttribute("data-value", value);

    const inputs = element.querySelectorAll("input");
    for (const input of inputs) {
      updateInputElement(input, Utils.EvalGetValue(value, input.getAttribute("data-key")));
    }
  }

  updateInputElement(element, value) {
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

  updateSelectElement(element, value) {
    const options = element.querySelectorAll("option");
    for (let i = 0; i < options.length; i++) {
      options[i].selected = options[i].value === value;
    }
  }

  updateImageElement(element, value) {
    element.src = value;
  }
}
globalThis.domUpdateHelper = new DOMUpdateHelper();

