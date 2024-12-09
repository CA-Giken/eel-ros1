/**
 * DOM Default UpdateCallback
 */
function updateDOM(element, value) {
  switch (typeof value) {
    case "boolean":
      element.checked = value;
      break;
    case "number":
      if (element.tagName === "INPUT") {
        element.value = value;
        break;
      }
      element.innerText = value;
      break;
    case "string":
      if (value.startsWith("data:image")) {
        element.src = value;
        break;
      }
      if (element.tagName === "INPUT") {
        element.value = value;
        break;
      }
      element.innerText = value;
      break;
    case "object":
      if (Array.isArray(value)) {
        inputs = element.querySelectorAll("input");
        length = Math.min(inputs.length, value.length);
        for (let i = 0; i < inputs.length; i++) {
          inputs[i].value = value[i];
        }
      }
      break;
    default:
      console.warn("[CA] Unexpected updateDOM type:", typeof value);
      break;
  }
}

/**
 * ROS PUBLISHER
 **/
const publishButtons = document.getElementsByClassName("publish");
for (const btn of publishButtons) {
  console.assert(btn.tagName === "BUTTON", "PublisherはButtonエレメントにのみ限定する");
  console.assert(btn.getAttribute("data-rtype") in MSG_TYPES, `Publisherのdata-rtypeが不正です. ${btn.getAttribute("data-rtype")}`);
  if (btn.tagName !== "BUTTON") {
    continue;
  }

  btn.addEventListener("click", async (e) => {
    const name = e.currentTarget.getAttribute("name");
    const type = MSG_TYPES[e.currentTarget.getAttribute("data-rtype")];
    const value = e.currentTarget.getAttribute("value");

    const event = new CustomEvent(ROS_EVENTS.Publish, {
      detail: { name, type, value },
    });
    document.dispatchEvent(event);
  });
}

/**
 * ROS SUBSCRIBER
 **/
const subscribeElements = document.getElementsByClassName("subscribe");
for (const element of subscribeElements) {
  console.assert(element.getAttribute("data-rtype") in MSG_TYPES, `Subscriberのdata-rtypeが不正です. ${element.getAttribute("data-rtype")}`);

  const name = element.getAttribute("name");
  const type = MSG_TYPES[element.getAttribute("data-rtype")];

  // Subscribe開始
  const event = new CustomEvent(ROS_EVENTS.Subscribe, {
    detail: { name, type },
  });
  document.dispatchEvent(event);

  // DOM更新コールバック登録
  if (element.hasAttribute("update-custom")) {
    // 特殊なDOM更新処理が必要な場合は、data-update-custom属性を設定する
    continue;
  }
  // Subscribeしたデータを受信した際のデフォルトDOM更新処理
  domUpdateHelper.registerCallback(element, (type, value) => {
    // MSG_TYPES -> js type に変換
    // valueは常にstring
    console.assert(type === MSG_TYPES[element.getAttribute("data-rtype")], "Typeが一致しません");
    var jsvalue;
    switch (type) {
      case MSG_TYPES.Bool:
        jsvalue = value === "True";
        break;
      case MSG_TYPES.Int32:
        jsvalue = parseInt(value);
        break;
      case MSG_TYPES.Int64:
        jsvalue = parseInt(value);
        break;
      case MSG_TYPES.Float32:
        jsvalue = parseFloat(value);
        break;
      case MSG_TYPES.Float64:
        jsvalue = parseFloat(value);
        break;
      case MSG_TYPES.String:
        jsvalue = value;
        break;
      case MSG_TYPES.Transform:
        jsvalue = value; // [x, y, z, qx, qy, qz, qw]
        break;
      case MSG_TYPES.Pose:
        jsvalue = value; // [x, y, z, qx, qy, qz, qw]
        break;
      case MSG_TYPES.Image:
        jsvalue = "data:image/jpeg;base64," + value; // base64string
        break;
      default:
        console.error("[CA] Unexpected ROS Message Type:", type);
        return;
    }
    // DOM更新
    updateDOM(element, value);
  });
}
document.addEventListener(ROS_EVENTS.SubscribedValue, async (e) => {
  const { name, type, value } = e.detail;
  const topicElements = document.getElementsByName(name);
  topicElements.forEach((element) => {
    domUpdateHelper.executeCallbacks(element, type, value);
  });
});



/**
 * ROS Parameters
 **/
const paramElements = document.getElementsByClassName("rosparam");
for (const element of paramElements) {
  console.assert(element.getAttribute("data-rtype") in PARAM_TYPES, `Parameterのdata-rtypeが不正です. ${element.getAttribute("data-rtype")}`);
  const name = element.getAttribute("name");
  const type = PARAM_TYPES[element.getAttribute("data-rtype")];
  const event = new CustomEvent(ROS_EVENTS.Param, {
    detail: { name, type },
  });
  document.dispatchEvent(event);
  
  // コールバック登録
  if (element.hasAttribute("update-custom")) {
    // 特殊なコールバックが必要な場合は、data-update-custom属性を設定する
    continue;
  }

  /** ROSPARAM SETTER */
  element.addEventListener("change", async (e) => {
    const name = e.currentTarget.getAttribute("name");
    const type = PARAM_TYPES[e.currentTarget.getAttribute("data-rtype")];
    const value = e.currentTarget.value;
    console.debug(name, type, value);
    const event = new CustomEvent(ROS_EVENTS.ParamSet, {
      detail: { name, type, value },
    });
    document.dispatchEvent(event);
  });

  // rosparamのデータを受信した際のデフォルトDOM更新処理
  domUpdateHelper.registerCallback(element, (type, value) => {
    // valueは常にstring
    console.assert(type === PARAM_TYPES[element.getAttribute("data-rtype")], "Typeが一致しません");
    var jsvalue;
    switch (type) {
      case PARAM_TYPES.Bool:
        jsvalue = value === "True";
        break;
      case PARAM_TYPES.Number:
        jsvalue = parseFloat(value);
        break;
      case PARAM_TYPES.String:
        jsvalue = value;
        break;
      default:
        console.error("[CA] Unexpected ROS Param Type:", type);
        return;
    }
    // DOM更新
    updateDOM(element, value);
  });
}

document.addEventListener(ROS_EVENTS.ParamUpdated, async (e) => {
  const { name, type, value } = e.detail;
  const paramElements = document.getElementsByName(name);
  paramElements.forEach((element) => {
    domUpdateHelper.executeCallbacks(element, type, value);
  });
});

/**
 * Unregister all ros pub/sub/params on unload page.
 **/
window.addEventListener("unload", async (e) => {
  for (const element of publishButtons) {
    eel.ros_unpublish(element.getAttribute("name"));
  }
  for (const element of subscribeElements) {
    eel.ros_unsubscribe(element.getAttribute("name"));
  }
  for (const element of paramElements) {
    eel.ros_unregister_param(element.getAttribute("name"));
  }
});