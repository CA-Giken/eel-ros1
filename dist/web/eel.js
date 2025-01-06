// Mock implementation of eel.js
window.eel = {
  // 基本的なプロキシ関数を作成するヘルパー
  _createProxy: function(funcName) {
    return function() {
      console.log(`Mock eel.${funcName} called with arguments:`, arguments);
      return Promise.resolve(null); // デフォルトでnullを返すPromise
    };
  },

  // expose関数 - Pythonの関数を公開する際に使用
  expose: function(func, name) {
    console.log(`Mock eel.expose: Exposing function ${name || func.name}`);
  },

  // set_host関数 - ホストURLを設定
  set_host: function(host) {
    console.log(`Mock eel.set_host: Setting host to ${host}`);
  },

  // Proxyハンドラを使用して動的に関数呼び出しに対応
  proxy: new Proxy({}, {
    get: function(target, prop) {
      if (!target[prop]) {
        target[prop] = window.eel._createProxy(prop);
      }
      return target[prop];
    }
  })
};

// Proxyを使用して未定義の関数呼び出しをハンドル
window.eel = new Proxy(window.eel, {
  get: function(target, prop) {
    if (prop in target) {
      return target[prop];
    }
    return target.proxy[prop];
  }
});

eel.proxy.ros_get_param = async function (name) {
  console.log(`Mock eel.ros_get_param: Getting ROS param ${name}`);
  return Promise.resolve([123, 8987.21, 0.018, 0.421]);
}