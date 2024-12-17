/**
 * イベントディスパッチャー追加のためのラッパーコンポーネント用関数
 */

function dispatchWrappedEvent(originalEvent, customEventName) {
  // イベントターゲットがevent-wrapperクラスを持っている場合はイベント発行を行わない
  const target = originalEvent.target;
  if (target.classList.contains("event-wrapper")) {
    return;
  }
  // // ラッパーコンポーネントに最も近いタグが持つdata-*を取得
  // const isWrappedElement = target.closest(".event-wrapper") !== null;
  // if (!isWrappedElement) return;

  const detail = { ...target.dataset };
  const customEvent = new CustomEvent(customEventName, {
    bubbles: true,
    detail,
  });
  document.dispatchEvent(customEvent);
}