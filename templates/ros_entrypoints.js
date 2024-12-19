globalThis.eelEntrypoints = {
  updateSubscribedValue: function(name, value) {
    const event = new CustomEvent(ROS_EVENTS.SubscribedValue, {
      detail: { name, value },
    });
    document.dispatchEvent(event);
  },
  updateParam: function(name, type, value) {
    const event = new CustomEvent(ROS_EVENTS.ParamUpdated, {
      detail: { name, type, value },
    });
    document.dispatchEvent(event);
  },
  health: function(value) {
    console.log("[CA] Python -> JS: OK");
    return document.title + ": " + value;
  }
}

eel.expose(eelEntrypoints.updateSubscribedValue, "updateSubscribedValue");
eel.expose(eelEntrypoints.updateParam, "updateParam");
eel.expose(eelEntrypoints.health, "health");