# Eel-ROS1 Reference

## Elements for Ros Param

class="rosparam"

<table>
<tr><th>Param型<th>要素<th>Widget<th>子要素<th>Attr<th>Mandatory</td>
<tr><td rowspan="2"><td colspan="3" rowspan="2">共通<td>class="rosparam"<td>Yes
<tr><td>name="param name"<td>Yes
<tr><td rowspan="8">number<td rowspan="6">input<td rowspan="4">Field<sup>(1)</sup><td rowspan="4">-<td>type="number"<td>
<tr><td>format="format string"<td>
<tr><td>max="const"<td>
<tr><td>min="const"<td>
<tr><td rowspan="2">Radio Button<td rowspan="2">-<td>type="radio"<td>Yes
<tr><td>value="const"<td>Yes
<tr><td rowspan="2">select<td rowspan="2">Combo Box<td><td>type="number"<td>
<tr><td>option<td>value="const"<td>Yes
<tr><td>bool<td>input<td>Check Box<td>-<td>type="checkbox"<td>Yes
<tr><td>string<td>input<td>Field<td>-<td>type="text"<td>Yes
<tr><td rowspan="3">object<td rowspan="3">div<td>-<td><td>type="number"<td>
<tr><td rowspan="2">Field<td rowspan="2">input<td colspan="2">Same as (1) number&rarr;input&rarr;Field
<tr><td>key="param name detailed"<td>Yes
<tr><td rowspan="3">list<td rowspan="3">div<td>-<td><td>type="number"<td>
<tr><td rowspan="2">Field<td rowspan="2">input<td colspan="2">Same as (1) number&rarr;input&rarr;Field
<tr><td>key="param name or address [...]"<td>Yes
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
  <input key="['doBin']" />
  <input key="['doICP']" />
</div>
~~~
下の表記も同じ意味です
~~~
<div class="rosparam" name="/solver">
  <input key=".doBin" />
  <input key=".doICP" />
</div>
~~~

5. /left/detector/ROIEdge[0]/Data[0..3]をField設定にする
~~~
<div class="rosparam" name="/left/detector/ROIEdge">
  <input key="[0]['Data'][0]" />
  <input key="[0]['Data'][1]" />
  <input key="[0]['Data'][2]" />
  <input key="[0]['Data'][3]" />
</div>
~~~

## Elements for Subscriber

## Elements for Publisher

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
    to: /latest/camera/image
~~~

3. ファイルを読み込んでトピックに再発行する  
Imageトピックの代わりにファイルを読み出してトピックとして再発行します。
~~~
config:
  cache:
  - file: /tmp/capt00.png
    to: /cache/camera/image    #このときは省略できない
~~~

4. 複数の表記  
YAMLリスト形式にて、複数のキャシュを設定します(1個しかないときもリスト形式で記述する)。

~~~
config:
  cache:
  - topic: /camera/image
  - topic: /camera/image
    to:  /latest/camera/image
  - file: /tmp/capt00.png
    to: /cache/camera/image
~~~
