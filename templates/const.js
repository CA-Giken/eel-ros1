globalThis.PARAM_TYPES = {
  Bool: "Bool",
  Number: "Number",
  String: "String",
  Object: "Object",
  Array: "Array",
}

globalThis.ROS_EVENTS = {
  Publish: "Publish", // { name, type, value },
  Subscribe: "Subscribe", // { name, type },
  SubscribedValue: "SubscribedValue", // { name, value },
  Param: "Param", // { name },
  ParamSet: "ParamSet", // { name, type, value },
  ParamUpdated: "ParamUpdated", // { name, value },
}