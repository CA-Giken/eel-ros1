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
  }
}