# UI App Boilerplate for ROS1
Well designed UI boilerplate with HTML+CSS for ROS1(Noetic), with Eel

## Installation

```
cd ~/catkin_ws/src
git clone https://github.com/CA-Giken/eel-example.git
cd ~/catkin_ws

pip install eel[jinja2]
apt update & apt install -y chromium-browser

catkin build
```

## Run

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch eel_example start.launch
```

## Development (Launch file)

- `--html_dir`: Directory for html files. default to `templates/`.
- `--port`: Port number the web page hosted at.

```
<launch>
    <node name="eel_ros1" pkg="eel_ros1" type="main.py" args="--html_dir=[templates_directory_path]" output="screen" />
</launch>
```

## Development (Logic)

Last published/subscribed values are stored in pubs/subs dicts at `src/eel_ros1/models/ros_service.py`.

```python:src/eel_ros1/models/ros_service.py
pubs = {} # { "/topic_name": { "publisher": rospy.Publisher, "last_value": value } }
subs = {} # { "/topic_name": { "subscriber": rospy.Subscriber, "last_value": value } }
```

ROS Params are stored in params dict at `src/eel_ros1_models/rosparam.py`.

## Development (UI)

HTML+CSS files are in `templates/` in default.

This directory can be changed by `--html_dir=[dir_path]` option on launch.

### Page

Pages must be `templates/aaa.html`, nested files are not reckoned as pages.

```html
<!DOCTYPE html>
<html>
  <head>
    <title>Page title</title>
    <link rel="stylesheet" type="text/css" href="/styles.css" />
    <script type="text/javascript" src="/eel.js"></script>
    <script type="text/javascript" src="/const.js"></script>
    <script type="text/javascript" src="/dom-helper.js"></script>
    <script type="text/javascript" src="/ros.js"></script>
  </head>
  <body>
    <!-- Content -->
    <script type="text/javascript" src="/scripts.js"></script> <!-- DOM内を走査してイベントをバインドするので最後に必要 -->
  </body>
</html>
```

Javascript and CSS files will be bundled if it locates just on `templates/` directory. Nested files won't be bundled.

#### Components

With Jinja2, you can use Template tags.

```html
  <h2>ROS Subscribers</h2>
  <h3>/eel_subtest/bool</h3>
  {% with name="/eel_subtest/bool" %}
    {% include "components/subscribers/Bool.html" %}
  {% endwith %}

  <h2>ROS Publishers</h2>
  <h3>/eel_pubtest/bool</h3>
  {% with name="/eel_pubtest/bool", value="True", label="Publish Bool" %}
    {% include "components/publishers/Bool.html" %}
  {% endwith %}

  <h2>ROS Params</h2>
  <h3>/eel_paramtest/bool</h3>
  {% with name="/eel_paramtest/bool" %}
    {% include "components/params/Bool.html" %}
  {% endwith %}
```

### Publisher

All publish values must be string or list of string in Javascript, converted to ROS Message type in Python.

```html
<button class="publish" name="/pub_name" data-rtype="String" value="Text">
  Label
</button>
```

| ROS Message Type | data-rtype | Value Example |
| --- | --- | --- |
| Bool:std_msgs | Bool | "True", "False" |
| Int32:std_msgs | Int32 | "0" |
| Int64:std_msgs | Int64 | "0" |
| Float32:std_msgs | Float32 | "0.0" |
| Float64:std_msgs | Float64 | "0.0" |
| String:std_msgs | String | "Text" |

### Subscriber

All subscribed values are passed as string or list of string to Javascript.

```html
<div class="subscribe" name="/sub_name" data-rtype="String"></div>
```

| ROS Message Type | data-rtype | Value Example |
| --- | --- | --- |
| Bool:std_msgs | Bool | "True", "False" |
| Int32:std_msgs | Int32 | "0" |
| Int64:std_msgs | Int64 | "0" |
| Float32:std_msgs | Float32 | "0.0" |
| Float64:std_msgs | Float64 | "0.0" |
| String:std_msgs | String | "Text" |
| Transform:geometry_msgs | Transform | [ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ] |
| Pose:geometry_msgs | Pose | [ "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" ] |
| Image:sensor_msgs | Image | base64string |

### Parameters

Values are passed as string, number or boolean depends on data-rtype

```html
<input class="rosparam" name="/param_name" type="checkbox" data-rtype="Bool" />
```

| ROS Param Type | data-rtype | Value Example |
| --- | --- | --- |
| string | String | text |
| float | Number | 0 |
| bool | Bool | true, false |

### Tabs

To prevent unintentional pub/sub lifecycle and display, iframe must be controlled by css `visible` state like below.

```html
<div class="container">
  <div class="component">
    <a name="tabs"><h2>Tabs</h2></a>
    <div class="tabs">
      <button class="tab" onclick="openTab(event, 'tab1')">Tab 1</button>
      <button class="tab" onclick="openTab(event, 'tab2')">Tab 2</button>
      <button class="tab" onclick="openTab(event, 'tab3')">Tab 3</button>
    </div>
    <iframe class="tabcontent" id="tab1" src="/frame1.html" style="display: none"></iframe>
    <iframe class="tabcontent" id="tab2" src="/frame2.html" style="display: none"></iframe>
    <iframe class="tabcontent" id="tab3" src="/frame3.html" style="display: none"></iframe>
  </div>
</div>
<script type="text/javascript">
  function openTab(evt, tabName) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
      tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tab");
    for (i = 0; i < tablinks.length; i++) {
      tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
  }
</script>
```

### Custom UI Development

Adding `update-custom` attribute besides ros related tag makes basic registration skipped.
```html
<input
  update-custom
  class="rosparam"
  data-rtype="String"
  type="radio"
  name="/eel_paramtest/string"
  value="test"
/>
```
Now you can manually bind to events descibed below.

#### CustomEvents

All Publisher, Subscriber, ROS Param events are dispatched in the document.
| Events | detail | Notes |
| --- | --- | --- |
| ROS_EVENTS.Publish | `{ name: string, type: string, value: string }` | init ros publisher and publish|
| ROS_EVENTS.Subscribe | `{ name: string, type: string }` | init ros subscriber |
| ROS_EVENTS.SubscribedValue | `{ name: string, type: string, value: string }` | when value updated |
| ROS_EVENTS.Param | `{ name: string, type: string }` | init ros param |
| ROS_EVENTS.ParamSet | `{ name: string, type: string, value: string }` | set ros param |
| ROS_EVENTS.ParamUpdated | `{ name: string, type: string, value: string }` | when ros param updated |

#### Custom UI Example

Example of select box rosparam ui:
```html:templates/components/params/EnumSelect.html
<!-- 
  Usage: 
    name: "/eel_paramtest/enum"
    map: 
      - value: "1"
        label: "One"
      - value: "2"
        label: "Two"
 -->
{% set element_id = generate_id() %}
<select
  update-custom
  class="rosparam"
  data-rtype="String"
  name="{{name}}"
  id="{{element_id}}"
  value=""
>
  {% for item in map %}
  <option value="{{item['value']}}">{{item['label']}}</option>
  {% endfor %}
</select>

<script>
  function updateEnumDOM(type, value){
    var element = document.getElementById("{{element_id}}");
    var options = element.querySelectorAll("option");
    options.forEach((option) => {
      option.selected = option.value === value;
    });
  }
  var element = document.getElementById("{{element_id}}");
  domUpdateHelper.registerCallback(element, updateEnumDOM);
  element.addEventListener("change", (event) => {
    const newEvent = new CustomEvent(ROS_EVENTS.ParamSet, {
      detail: {
        name: event.target.getAttribute("name"),
        type: PARAM_TYPES[event.target.getAttribute("data-rtype")],
        value: event.target.value
      }
    });
    document.dispatchEvent(newEvent);
  });
</script>
```

#### UI Testing

UI testing functions are in `templates/tester.js`.

Uncomment the last line with `// mock_ros();` to monitor pub/sub/params registration with console.log.

`pub_event()`, `sub_event()`, `param_event()` can be called from Google Chrome DevTools.