globalThis.MSG_TYPES = {
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
  SubscribedValue: "SubscribedValue", // { name, type, value },
  Param: "Param", // { name, type },
  ParamSet: "ParamSet", // { name, type, value },
  ParamUpdated: "ParamUpdated", // { name, value },
}