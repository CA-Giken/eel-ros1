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

```
<launch>
    <node name="eel_example" pkg="eel_example" type="main.py" args="--html_dir=[templates_directory_path]" output="screen" />
</launch>
```

## Development (Logic)

Python files are in `src/eel_example`.

Last published/subscribed values are stored in pubs/subs dicts at `src/eel_example/models/ros_service.py`.

```python:src/eel_example/models/ros_service.py
Config = {
    "package_name": "eel_example", # TODO: Modify to your package name
    "log_level": "info", # debug | info | warn | error
}
```

```python:src/eel_example/models/ros_service.py
pubs = {} # { "/topic_name": { "publisher": rospy.Publisher, "last_value": value } }
subs = {} # { "/topic_name": { "subscriber": rospy.Subscriber, "last_value": value } }
```

If managing ROS Pub/Sub internally, `src/eel_example/models/ros_service.py` is recommended part to modify.

## Development (UI)

HTML+CSS files are in `templates/` in default.

This directory can be changed by `--html_dir=[dir_path]` option on launch.

### Page

Pages must be `templates/aaa.html`, nested files are not reckoned as pages.

```html
<!-- templates/aaa.html -->
<!DOCTYPE html>
<html>
  <head>
    <title>Title</title>
    <link rel="stylesheet" type="text/css" href="/styles.css" />
  </head>
  <body>
    <!-- contents -->
    <script type="text/javascript" src="/eel.js"></script>
    <script type="text/javascript" src="/scripts.js"></script> <!-- Must be located at the last of the body -->
  </body>
</html>
```

Javascript and CSS files will be bundled if it locates just on `templates/` directory. Nested files won't be bundled.

#### Components

With Jinja2, you can use Template tags.

```html
<div class="container">
  <div class="component">
    <h2>ROS Subscribers</h2>
    <h3>/eel_subtest/bool</h3>
    {% with name="/eel_subtest/bool" %} {% include
    "components/subscribers/Bool.html" %} {% endwith %}
  </div>
</div>

<div class="container">
  <div class="component">
    <h2>ROS Publishers</h2>
    <h3>/eel_pubtest/bool</h3>
    {% with name="/eel_pubtest/bool", value="True", label="Publish Bool"
    %} {% include "components/publishers/Bool.html" %} {% endwith %}
  </div>
</div>
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