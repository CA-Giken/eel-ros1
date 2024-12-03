const MSG_TYPES = {
  // Bool = "Bool";
  // Int32 = "Int32";
  // Int64 = "Int64";
  // Float32 = "Float32";
  // Float64 = "Float64";
  // String = "String";
  // Transform = "Transform";
  // Pose = "Pose";
  // Image = "Image";
  Bool: "Bool:std_msgs",
  Int32: "Int32:std_msgs",
  Int64: "Int64:std_msgs",
  Float32: "Float32:std_msgs",
  Float64: "Float64:std_msgs",
  String: "String:std_msgs",
  Transform: "Transform:geometry_msgs",
  Pose: "Pose:geometry_msgs",
  Image: "Image:sensor_mags",
};

/**
 * ROS PUBLISHER
 **/
const publishButtons = document.getElementsByClassName("publish");
console.debug("publisher", document.title, publishButtons);
for (const btn of publishButtons) {
  // PublisherはButtonエレメントにのみ限定する
  if (btn.tagName !== "BUTTON") {
    continue;
  }

  btn.addEventListener("click", async (e) => {
    const rostopic_name = btn.getAttribute("name");
    const type = btn.getAttribute("data-rtype");
    const value = btn.getAttribute("value");
    console.debug(rostopic_name, type, value);
    eel.ros_publish(rostopic_name, type, value);
  });
}

/**
 * ROS SUBSCRIBER
 **/
const subscribeElements = document.getElementsByClassName("subscribe");
console.debug("subscriber", document.title, subscribeElements);
for (const element of subscribeElements) {
  const topic_name = element.getAttribute("name");
  const type = element.getAttribute("data-rtype");
  eel.ros_subscribe(topic_name, type);
}

eel.expose(updateSubscribedValue);
function updateSubscribedValue(topicName, type, value) {
  const topicElements = document.getElementsByName(topicName);
  for (const element of topicElements) {
    switch (type) {
      case MSG_TYPES.Bool:
        element.innerText = value;
        break;
      case MSG_TYPES.Int32:
        element.innerText = value;
        break;
      case MSG_TYPES.Int64:
        element.innerText = value;
        break;
      case MSG_TYPES.Float32:
        element.innerText = value;
        break;
      case MSG_TYPES.Float64:
        element.innerText = value;
        break;
      case MSG_TYPES.String:
        element.innerText = value;
        break;
      case MSG_TYPES.Transform:
        // valueは配列が来る事を期待
        inputs = element.querySelectorAll("input");
        length = Math.min(inputs.length, value.length);
        for (let i = 0; i < inputs.length; i++) {
          inputs[i].value = value[i];
        }
        break;
      case MSG_TYPES.Pose:
        // valueは配列が来る事を期待
        inputs = element.querySelectorAll("input");
        length = Math.min(inputs.length, value.length);
        for (let i = 0; i < inputs.length; i++) {
          inputs[i].value = value[i];
        }
        break;
      case MSG_TYPES.Image:
        img = element.querySelector("img");
        img.src = "data:image/jpeg;base64," + value;
        break;
      default:
        console.error("[CA] Unexpected ROS Message type:", type);
        break;
    }
  }
}

/**
 * ROS Parameters
 **/
const paramInputs = document.getElementsByClassName("rosparam");
console.debug("params", document.title, paramInputs);
for (const input of paramInputs) {
  eel.ros_register_param(input.getAttribute("name"), input.getAttribute("data-rtype"));
}
for (const input of paramInputs) {
  if(input.getAttribute("type") === "checkbox"){
    input.addEventListener("change", async (e) => {
      const name = input.getAttribute("name");
      const type = input.getAttribute("data-rtype");
      const value = input.checked;
      console.debug(name, type, value);
      eel.ros_set_param(name, type, value);
    });
    continue;
  }
  /** ROSPARAM SETTER */
  input.addEventListener("blur", async (e) => {
    const name = input.getAttribute("name");
    const type = input.getAttribute("data-rtype");
    const value = input.getAttribute("value");
    console.debug(name, type, value);
    eel.ros_set_param(name, type, value);
  });
}

/** ROSPARAM GETTER */
eel.expose(updateParam);
function updateParam(param_name, value) {
  console.log("[CA] updateParam", param_name, value);
  targetInputs = document.getElementsByName(param_name);
  console.debug("targetInputs", targetInputs);
  for (const input of targetInputs) {
    if (typeof value == "string") {
      input.value = value;
    } else if (typeof value == "boolean") {
      input.checked = value;
    } else if (typeof value == "number") {
      input.value = value;
    } else {
      console.error(
        `[CA] Rosparam input view only accepts string, boolean, number: received ${typeof value}`
      );
    }
  }
}

/**
 * Unregister all events on onload page.
 **/
window.addEventListener("unload", async (e) => {
  for (const input of paramInputs) {
    eel.ros_unregister_param(input.getAttribute("name"));
  }
  for (const element of subscribeElements) {
    eel.ros_unsubscribe(element.getAttribute("name"));
  }
});

// Health check to notify successful js runtime.
eel.expose(health);
function health(value) {
  console.log("[CA] Python -> JS: OK");
  return document.title + ": " + value;
}
eel.health("success");
