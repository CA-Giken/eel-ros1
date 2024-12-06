/**
 * ROS PUBLISHER
 * カスタムイベントPublishEvent を購読して、ROSにPublishする
 **/
document.addEventListener(ROS_EVENTS.Publish, async (e) => {
  const { topicName, type, value } = e.detail;
  eel.ros_publish(topicName, type, value);
});

/**
 * ROS SUBSCRIBER
 * カスタムイベントSubscribeEvent を購読して、ROSにSubscribeする
 * サブスクライバーが受信したデータをカスタムイベントSubscribedValue で通知する
 **/
document.addEventListener(ROS_EVENTS.Subscribe, async (e) => {
  const { topicName, type } = e.detail;
  eel.ros_subscribe(topicName, type);
});

eel.expose(updateSubscribedValue);
function updateSubscribedValue(topicName, type, value) {
  const event = new CustomEvent(ROS_EVENTS.SubscribedValue, {
    detail: { topicName, type, value },
  })
  document.dispatchEvent(event);
}

/**
 * ROS Parameters
 * カスタムイベントParamEvent を購読して、ROSにパラメータを登録する
 **/
document.addEventListener(ROS_EVENTS.Param, async (e) => {
  const { paramName, type } = e.detail;
  eel.ros_register_param(paramName, type);
});

document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
  const { paramName, type, value } = e.detail;
  eel.ros_set_param(paramName, type, value);
});

/** ROSPARAM GETTER */
eel.expose(updateParam);
function updateParam(paramName, type, value) {
  const event = new CustomEvent(ROS_EVENTS.ParamUpdated, {
    detail: { paramName, type, value },
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
