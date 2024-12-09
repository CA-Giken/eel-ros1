class DOMUpdateHelper {
  constructor() {
    this.updateCallbacks = new WeakMap();
  }

  registerCallback(element, callback) {
    if (this.updateCallbacks.has(element)) {
      this.updateCallbacks.get(element).push(callback);
    } else {
      this.updateCallbacks.set(element, [callback]);
    }
  }

  executeCallbacks(element, type, value) {
    if (this.updateCallbacks.has(element)) {
      this.updateCallbacks.get(element).forEach((callback) => {
        callback(type, value);
      });
    }
  }
}
globalThis.domUpdateHelper = new DOMUpdateHelper();