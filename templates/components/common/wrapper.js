/**
 * イベントディスパッチャー追加のためのラッパーコンポーネント用関数
 */

function dispatchWrappedEvent(originalEvent, customEventName) {
  const currentTarget = originalEvent.currentTarget;

  // 現在のターゲットから最も近い非event-wrapper要素を見つける関数
  function findClosestNonWrapper(element) {
    // 要素自体がevent-wrapperでない場合はその要素を返す
    if (!element.classList.contains('event-wrapper')) {
      return element;
    }
    // 子要素を走査
    for (const child of element.children) {
      const result = findClosestNonWrapper(child);
      if (result) return result;
    }
    return null;
  }
  const wrappedElement = findClosestNonWrapper(currentTarget);
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