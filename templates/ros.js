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

eel.expose(updateSubscribedValue);
function updateSubscribedValue(name, type, value) {
  const event = new CustomEvent(ROS_EVENTS.SubscribedValue, {
    detail: { name, type, value },
  })
  document.dispatchEvent(event);
}

/**
 * ROS Parameters
 * カスタムイベントParamEvent を購読して、ROSにパラメータを登録する
 **/
document.addEventListener(ROS_EVENTS.Param, async (e) => {
  const { name, type } = e.detail;
  eel.ros_register_param(name, type);
});

document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
  const { name, type, value } = e.detail;
  eel.ros_set_param(name, type, value);
});

/** ROSPARAM GETTER */
eel.expose(updateParam);
function updateParam(name, type, value) {
  const event = new CustomEvent(ROS_EVENTS.ParamUpdated, {
    detail: { name, type, value },
  });
  document.dispatchEvent(event);
}

// Health check to notify successful js runtime.
eel.expose(health);
function health(value) {
  console.log("[CA] Python -> JS: OK");
  return document.title + ": " + value;
}
eel.health(document.title);
