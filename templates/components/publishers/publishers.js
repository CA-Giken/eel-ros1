/**
 * ROS PUBLISHER
 **/
document.addEventListener("DOMContentLoaded", async () => {
  /**
   * Publisherの必須属性
   * - class="publish"
   * - name="[/topic_name]"
   * - data-rtype="[MSG_TYPES]"
   * - value="[value]"
   */
  const publishButtons = document.getElementsByClassName("publish");
  for (const btn of publishButtons) {
    /**
     * HTMLタグバリデータ
     */
    console.assert(btn.tagName === "BUTTON", "PublisherはButtonエレメントにのみ限定する");
    console.assert(btn.getAttribute("name") !== null, "Publisherのname属性が必要です.");
    console.assert(btn.getAttribute("data-rtype") in MSG_TYPES, `Publisherのdata-rtypeが不正です. ${btn.getAttribute("data-rtype")}`);
    if (btn.tagName !== "BUTTON") {
      continue;
    }

    btn.addEventListener("click", async (e) => {
      const name = e.currentTarget.getAttribute("name");
      const type = MSG_TYPES[e.currentTarget.getAttribute("data-rtype")];
      const value = e.currentTarget.getAttribute("value");

      const event = new CustomEvent(ROS_EVENTS.Publish, {
        detail: { name, type, value },
      });
      document.dispatchEvent(event);
    });
  }
});