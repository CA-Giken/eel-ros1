/**
 * Unregister all ros pub/sub/params on unload page.
 **/
window.addEventListener("unload", async (e) => {
  for (const element of publishButtons) {
    eel.ros_unpublish(element.getAttribute("name"));
  }
  for (const element of subscribeElements) {
    eel.ros_unsubscribe(element.getAttribute("name"));
  }
  for (const element of paramElements) {
    eel.ros_unregister_param(element.getAttribute("name"));
  }
});