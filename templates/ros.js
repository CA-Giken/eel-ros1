document.addEventListener("DOMContentLoaded", async () => {
  /**
   * ROS PUBLISHER
   * カスタムイベントPublishEvent を購読して、ROSにPublishする
   **/
  document.addEventListener(ROS_EVENTS.Publish, async (e) => {
    const { name, type, value } = e.detail;
    eel.ros_publish(name, type, value);
  });

  /**
   * ROS SUBSCRIBER
   * カスタムイベントSubscribeEvent を購読して、ROSにSubscribeする
   * サブスクライバーが受信したデータをカスタムイベントSubscribedValue で通知する
   **/
  document.addEventListener(ROS_EVENTS.Subscribe, async (e) => {
    const { name, type } = e.detail;
    eel.ros_subscribe(name, type);
  });

  /**
   * ROS Parameters
   * カスタムイベントParamEvent を購読して、ROSにパラメータを登録する
   **/
  document.addEventListener(ROS_EVENTS.Param, async (e) => {
    const { name } = e.detail;
    eel.ros_register_param(name);
  });

  /**
   * ROS Parameters Setter
   * カスタムイベントParamSet を購読して、ROSのパラメータを設定
   */
  document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
    const { name, type, value } = e.detail;
    eel.ros_set_param(name, type, value);
  });

  eel.expose(updateSubscribedValue);
  function updateSubscribedValue(name, type, value) {

    const event = new CustomEvent(ROS_EVENTS.SubscribedValue, {
      detail: { name, type, value: to_jsvalue(type, value) },
    });
    document.dispatchEvent(event);
  }

  eel.expose(updateParam);
  function updateParam(name, type, value) {
    const event = new CustomEvent(ROS_EVENTS.ParamUpdated, {
      detail: { name, type, value: to_jsvalue(type, value) },
    });
    document.dispatchEvent(event);
  }

  /**
   * ヘルスチェック
   */
  eel.health(document.title);

  eel.expose(health);
  function health(value) {
    console.log("[CA] Python -> JS: OK");
    return document.title + ": " + value;
  }
});

globalThis.ROSInterface = {
  getParam: async function(name) {
    return await eel.ros_get_param(name);
  }
}

// Python -> JS のWebSocket通信データは全てstringか配列stringで送られてくる
function to_jsvalue(type, value){
  if (type === MSG_TYPES.Float32) {
    return parseFloat(value);
  }
  if (type === MSG_TYPES.Float64) {
    return parseFloat(value);
  }
  if (type === MSG_TYPES.Int32) {
    return parseInt(value);
  }
  if (type === MSG_TYPES.Int64) {
    return parseInt(value);
  }
  if (type === MSG_TYPES.String) {
    return value;
  }
  if (type === MSG_TYPES.Bool) {
    return value === "True";
  }
  if (type === MSG_TYPES.Transform) {
    return value.map(parseFloat);
  }
  if (type === MSG_TYPES.Pose) {
    return value.map(parseFloat);
  }
  if (type === MSG_TYPES.Image) {
    // base64string
    return "data:image/jpeg;base64," + value;
  }
  if (type === PARAM_TYPES.Bool) {
    return value === "True";
  }
  if (type === PARAM_TYPES.Number) {
    return parseFloat(value);
  }
  if (type === PARAM_TYPES.String) {
    return value;
  }
  throw new Error("Unknown type: " + type);
}