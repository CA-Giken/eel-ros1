document.addEventListener("DOMContentLoaded", async () => {
  /**
   * ROS PUBLISHER
   * カスタムイベントPublishEvent を購読して、ROSにPublishする
   **/
  document.addEventListener(ROS_EVENTS.Publish, async (e) => {
    const { name, type, value } = e.detail;
    console.debug(`[CA] Publish: ${name} (${type}, ${value})`);
    eel.ros_publish(name, type, value);
  });

  /**
   * ROS SUBSCRIBER
   * カスタムイベントSubscribeEvent を購読して、ROSにSubscribeする
   * サブスクライバーが受信したデータをカスタムイベントSubscribedValue で通知する
   **/
  document.addEventListener(ROS_EVENTS.Subscribe, async (e) => {
    const { name, type } = e.detail;
    console.debug(`[CA] Subscribe: ${name} (${type})`);
    eel.ros_subscribe(name, type);
  });

  /**
   * ROS Parameters
   * カスタムイベントParamEvent を購読して、ROSにパラメータを登録する
   **/
  document.addEventListener(ROS_EVENTS.Param, async (e) => {
    const { name } = e.detail;
    console.debug(`[CA] Param: ${name}`);
    eel.ros_register_param(name);
  });

  /**
   * ROS Parameters Setter
   * カスタムイベントParamSet を購読して、ROSのパラメータを設定
   */
  document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
    const { name, type, value } = e.detail;
    console.debug(`[CA] ParamSet: ${name} (${type}, ${value})`);
    eel.ros_set_param(name, type, value);
  });

  /**
   * ヘルスチェック
  */
  eel.health(document.title);
});


globalThis.ROSInterface = {
  getParam: async function(name) {
    return await eel.ros_get_param(name);
  }
}

