/**
 * This is only for testing UI
 */

/** Mocking ROS Layer */
function mock_ros() {
  document.addEventListener(ROS_EVENTS.Publish, async (e) => {
    const { name, type, value } = e.detail;
    console.log(`[CA] PublishEvent: ${name}, ${type}, ${value}`);
  });

  document.addEventListener(ROS_EVENTS.Subscribe, async (e) => {
    const { name, type } = e.detail;
    console.log(`[CA] SubscribeEvent: ${name}, ${type}`);
  });
  
  document.addEventListener(ROS_EVENTS.Param, async (e) => {
    const { name, type } = e.detail;
    console.log(`[CA] ParamEvent: ${name}, ${type}`);
  });

  document.addEventListener(ROS_EVENTS.ParamSet, async (e) => {
    const { name, type, value } = e.detail;
    console.log(`[CA] ParamSetEvent: ${name}, ${type}, ${value}`);
    param_event(name, type);
  });
}

function pub_event(name, type, value) {
  const event = new CustomEvent(ROS_EVENTS.Publish, {
    detail: { name, type, value },
  });
  document.dispatchEvent(event);
}

function sub_event(name, type) {
  const event = new CustomEvent(ROS_EVENTS.Subscribe, {
    detail: { name, type },
  });
  document.dispatchEvent(event);
}

function param_event(name, type) {
  const event = new CustomEvent(ROS_EVENTS.Param, {
    detail: { name, type },
  });
  document.dispatchEvent(event);
}

// mock_ros();