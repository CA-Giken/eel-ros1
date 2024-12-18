# Eel-ROS1 Reference

## Elements for Ros Param

class="rosparam"

<table>
<tr><th>Param型<th>要素<th>Widget<th>子要素<th>Attr<th>Mandatory</td>
<tr><td rowspan="2"><td colspan="3" rowspan="2">共通<td>class="rosparam"<td>Yes
<tr><td>name={param name}<td>Yes
<tr><td rowspan="8">number<td rowspan="6">input<td rowspan="4">Field<sup>(1)</sup><td rowspan="4">-<td>type="number"<td>
<tr><td>format={format string}<td>
<tr><td>max={const number}<td>
<tr><td>min={const number}<td>
<tr><td rowspan="2">Radio Button<td rowspan="2">-<td>type="radio"<td>Yes
<tr><td>value={const number}<td>Yes
<tr><td rowspan="2">select<td rowspan="2">Combo Box<td><td>type="number"<td>
<tr><td>option<td>value={const number}<td>Yes
<tr><td>bool<td>input<td>Check Box<td>-<td>type="checkbox"<td>Yes
<tr><td>string<td>input<td>Field<td>-<td>type="text"<td>Yes
<tr><td rowspan="3">object<br>comprising "[...]"<td rowspan="3">div<td>-<td><td>type="number"<td>
<tr><td rowspan="2">Field<td rowspan="2">input<td colspan="2">Same as (1) number&rarr;input&rarr;Field
<tr><td>ext={name extension}<td>Yes
</table>

### Example

1. /solver/doBinをField設定にする
~~~
<input class="rosparam" name="/solver/doBin" />
~~~

2. /solver/doBinをRadio Button設定(1..3)にする
~~~
<input class="rosparam" type="radio" value="1" name="/solver/doBin"/>
<input class="rosparam" type="radio" value="2" name="/solver/doBin"/> 
<input class="rosparam" type="radio" value="3" name="/solver/doBin"/> 
~~~

3. /solver/doBinをCombo Box設定(1..3)にする
~~~
<select class="rosparam" name="/solver/doBin">
  <option value="1">固定二値化</option>
  <option value="2">適応二値化</option>
  <option value="3">大津二値化</option>
</select>
~~~

4. /solver/doBin,/solver/doICPをField設定にする
~~~
<div class="rosparam" name="/solver">
  <input ext=".doBin" />
  <input ext=".doICP" />
</div>
~~~
下の表記も可能です。
~~~
<div class="rosparam" name="/solver">
  <input ext="['doBin']" />
  <input ext="['doICP']" />
</div>
~~~

5. /left/detector/ROIEdge[0]/Data[0..3]をField設定にする
~~~
<div class="rosparam" name="/left/detector/ROIEdge">
  <input ext="[0].Data[0]" />
  <input ext="[0].Data[1]" />
  <input ext="[0].Data[2]" />
  <input ext="[0].Data[3]" />
</div>
~~~
または
~~~
<div class="rosparam" name="/left/detector/ROIEdge">
  <input ext="[0]['Data'][0]" />
  <input ext="[0]['Data'][1]" />
  <input ext="[0]['Data'][2]" />
  <input ext="[0]['Data'][3]" />
</div>
~~~
## Elements for Subscriber
<table>
<tr><th>Message型<th>(親)要素<th>Widget<th>子要素<th>Attr<th>Mandatory</td>
<tr><td rowspan="2"><td colspan="3" rowspan="2">All<td>class="subscribe"<td>Yes
<tr><td>name={topic name}<td>Yes
<tr><td rowspan="3">Any<br>except below<td rowspan="3">div<td><td>-<td>m-type={ros message type}<td>Yes
<tr><td rowspan="2">Field<td rowspan="2">input<td>ext={name extension}<td>Yes
<tr><td>format={format string}
<tr><td>Image<td>img<td><td>-<td>m-type="sensor_msgs/Image"<td>Yes
</table>

### Example

1. String型トピック/errorをFieldに表示
~~~
<div class="subscribe" m-type="std_msgs/String" name="/error">
  <input ext=".data" readonly />
</div>
~~~

2. TransformStamped型トピック/robot/toolの、frame名と直交座標を表示
~~~
<div class="subscribe" m-type="geometry_msgs/TransformStamped" name="/robot/tool">
  <input ext=".header.frame_id" readonly />
  <input ext=".transform.translation.x" readonly />
  <input ext=".transform.translation.y" readonly />
  <input ext=".transform.translation.z" readonly />
</div>
~~~

## Elements for Publisher
<table>
<tr><th>Message型<th>(親)要素<th>Widget<th>子要素<th>Attr<th>Mandatory</td>
<tr><td rowspan="2"><td colspan="3" rowspan="2">All<td>class="publish"<td>Yes
<tr><td>name={topic name}<td>Yes
<tr><td rowspan="5">Any<td rowspan="5">div<td rowspan="2"><td rowspan="2">-<td>m-type={ros message type}<td>Yes
<tr><td>rate={const number}
<tr><td rowspan="2">Field<td rowspan="2">input<td>ext={name extension}<td>Yes
<tr><td>format={format string}
<tr><td>Button<td>button<td><td>
</table>

1. Bool型トピック/solve/execにTrue:Boolを発行
~~~
<div class="publish" m-type="std_msgs/Bool" name="/solve/exec">
  <input ext=".data" value="true" type="hidden" />
  <button>解析実行</button>
</div>
~~~

2. Int型トピック/solve/thresholdにキー入力数値:Int32を発行
~~~
<div class="publish" m-type="std_msgs/Int32" name="/solve/threshold">
  <input ext=".data" type="number" />
  <button>整数値送信</button>
</div>
~~~

3. Transform型トピック/solve/tfにX座標のみキー入力数値、その他は固定値を発行
~~~
<div class="publish" m-type="geometry_msgs/Transform" name="/solve/tf">
  <input ext=".tranlation.x" type="number" />
  <input ext=".tranlation.y" value="1.0" type="hidden" />
  <input ext=".tranlation.z" value="2.0" type="hidden" />
  <input ext=".rotation.x" value="0.0" type="hidden" />
  <input ext=".rotation.y" value="0.0" type="hidden" />
  <input ext=".rotation.z" value="0.0" type="hidden" />
  <input ext=".rotation.w" value="0.0" type="hidden" />
  <button>X座標送信</button>
</div>
~~~

4. 定周期トピック発行(ボタンなし)
~~~
<div class="publish" m-type="std_msgs/Bool" name="/solve/exec" rate="1">
  <input ext=".data" value="true" type="hidden" />
</div>
~~~


## Appendix. 外部データファイル参照
CSVファイルなどから、キー抽出したテキストを表示する機能

## Appendix. Cache Layer
### Configuration Example
1. Imageトピックをキャッシュ  
Imageトピック/camera/imageを購読し、/cache/camera/imageにpublishするのと同時にキャシュに保持する。  
/cache/reload:Boolがpublishされたら、キャシュしているImageを/cache/camera/imageに、1回だけ再発行する。
~~~
config:
  cache:
  - topic: /camera/image
    m-type: sensor_msgs/Image
~~~

2. デフォルトpublish名を変える  
キャシュされたトピックのデフォルトの再発行名は、
~~~
/cache/<購読するトピック名>
~~~
となるが、下例のように変更可能(/latest/camera/imageに変更)。
~~~
config:
  cache:
  - topic: /camera/image
    m-type: sensor_msgs/Image
    to: /latest/camera/image
~~~

3. ファイルを読み込んでトピックに再発行する  
Imageトピックの代わりにファイルを読み出してトピックとして再発行します。
~~~
config:
  cache:
  - file: /tmp/capt00.png
    m-type: sensor_msgs/Image
    to: /cache/camera/image    #このときは省略できない
~~~

4. 複数の表記  
YAMLリスト形式にて、複数のキャシュを設定します(1個しかないときもリスト形式で記述する)。

~~~
config:
  cache:
  - topic: /camera/image
    m-type: sensor_msgs/Image
  - topic: /camera/image
    m-type: sensor_msgs/Image
    to:  /latest/camera/image
  - file: /tmp/capt00.png
    m-type: sensor_msgs/Image
    to: /cache/camera/image
~~~
