/**
 * ROS PUBLISHER
 **/
document.addEventListener("DOMContentLoaded", async () => {
  /**
   * Publisherの必須属性
   * - class="publish"
   * - name="[/topic_name]"
   * - ext="ROS_MESSAGE"
   * - value="[value]"
   */
  const publishes = document.getElementsByClassName("publish");
  for (const pub of publishes) {
    /**
     * HTMLタグバリデータ
     */
    const name = pub.getAttribute("name");
    console.assert(name, "Publisherのname属性が必要です");
    const type = pub.getAttribute("m-type");
    console.assert(type, `Publisherのext属性が必要です`);
    console.assert(pub.tagName === "DIV", "PublisherはDIVタグである必要があります");
    if (pub.tagName !== "DIV") {
      continue;
    }
    const btn = pub.querySelector("button");
    console.assert(btn, "Publisherはbuttonタグを含む必要があります");
    const inputs = pub.querySelectorAll("input");
    console.assert(inputs.length > 0, "Publisherはinputタグを含む必要があります");


    btn.addEventListener("click", async (e) => {
      let value = {};
      // inputタグのvalueを取得して辞書化
      inputs.forEach((input) => {
        const ext = input.getAttribute("ext");
        if (!ext) return;
        var type_asserted;
        if(input.type == "checkbox"){
          type_asserted = input.checked;
        } else if(input.type == "number"){
          type_asserted = Number(input.value);
        } else if(input.type == "radio"){
          type_asserted = Number(input.value);
        } else {
          type_asserted = input.value;
        }
        value = Utils.dictFullPathAdd(value, ext, type_asserted);
      });
      const event = new CustomEvent(ROS_EVENTS.Publish, {
        detail: { name, type, value },
      });
      document.dispatchEvent(event);
    });
  }
});