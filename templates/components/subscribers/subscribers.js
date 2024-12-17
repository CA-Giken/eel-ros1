/**
 * ROS SUBSCRIBER
 **/
document.addEventListener("DOMContentLoaded", async () => {
  /**
   * Subscriberの必須属性
   *  - class="subscribe"
   *  - name="[/topic_name]"
   *  - data-rtype="[MSG_TYPES]"
   */
  const subscribeElements = document.getElementsByClassName("subscribe");
  for (const element of subscribeElements) {
    /**
     * HTMLタグバリデータ
     */
    console.assert(element.getAttribute("name") !== null, "Subscriberのname属性が必要です.");
    console.assert(element.getAttribute("data-rtype") in MSG_TYPES, `Subscriberのdata-rtypeが不正です. ${element.getAttribute("data-rtype")}`);
    const name = element.getAttribute("name");
    const type = MSG_TYPES[element.getAttribute("data-rtype")];

    /**
     * SubscriberをROSに登録
     */
    const event = new CustomEvent(ROS_EVENTS.Subscribe, {
      detail: { name, type },
    });
    document.dispatchEvent(event);

    /**
     * DOM更新方法の登録
     */
    if (element.hasAttribute("update-custom")) {
      // 特殊なDOM更新処理が必要な場合は、data-update-custom属性を設定する
      continue;
    }

    // Subscribeしたデータを受信した際のデフォルトDOM更新処理
    domUpdateHelper.registerCallback(element, domUpdateHelper.updateElement);
  }

  /**
   * DOM更新イベントの登録
   */
  document.addEventListener(ROS_EVENTS.SubscribedValue, async (e) => {
    const { name, type, value } = e.detail;
    const topicElements = document.getElementsByName(name);
    topicElements.forEach((element) => {
      domUpdateHelper.executeCallbacks(element, { type, value });
    });
  });
});