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
  String: "String"
}

globalThis.ROS_EVENTS = {
  Publish: "Publish", // { topicName, type, value },
  Subscribe: "Subscribe", // { topicName, type },
  SubscribedValue: "SubscribedValue", // { topicName, type, value },
  Param: "Param", // { paramName, type },
  ParamSet: "ParamSet", // { paramName, type, value },
  ParamUpdated: "ParamUpdated", // { paramName, value },
}