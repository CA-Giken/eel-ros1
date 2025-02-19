document.addEventListener("DOMContentLoaded", (event) => {
  var elements = document.getElementsByClassName("filebrowser");
  for (const el of elements) {
    /** HTMLタグバリデータ */
    const buttonElement = el.querySelector("button");
    const inputElement = el.querySelector("input");
    console.assert(buttonElement, "FileBrowserはbuttonタグを含む必要があります");
    console.assert(inputElement, "FileBrowserはinputタグを含む必要があります");
    buttonElement.addEventListener("click", async (event) => {
      const filepath = await eel.open_filebrowser()();
      inputElement.value = filepath;
    });
  }
});