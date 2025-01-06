/**
 * イベントディスパッチャー追加のためのラッパーコンポーネント用関数
 */

function dispatchWrappedEvent(originalEvent, customEventName) {
  // イベントターゲットがevent-wrapperクラスを持っている場合はイベント発行を行わない
  const target = originalEvent.currentTarget;
  // ラッパーコンポーネントに最も近いタグが持つdata-*を取得
  const wrappedElement = target.closest(".event-wrapper") !== null;
  if (!wrappedElement) return;

  const detail = { ...wrappedElement.dataset };
  const customEvent = new CustomEvent(customEventName, {
    bubbles: true,
    detail,
  });
  document.dispatchEvent(customEvent);
}

document.addEventListener("DOMContentLoaded", () => {
  const clickWrappers = document.getElementsByClassName("click-wrapper");
  for(const wrapper of clickWrappers) {
    const eventName = wrapper.dataset.eventName;
    wrapper.addEventListener("click", (e) => dispatchWrappedEvent(e, eventName));
  }
  const hoverWrappers = document.getElementsByClassName("hover-wrapper");
  for(const wrapper of hoverWrappers) {
    const eventName = wrapper.dataset.eventName;
    wrapper.addEventListener("mouseover", (e) => dispatchWrappedEvent(e, eventName));
  }
  const leaveWrappers = document.getElementsByClassName("leave-wrapper");
  for(const wrapper of leaveWrappers) {
    const eventName = wrapper.dataset.eventName;
    wrapper.addEventListener("mouseout", (e) => dispatchWrappedEvent(e, eventName));
  }
});