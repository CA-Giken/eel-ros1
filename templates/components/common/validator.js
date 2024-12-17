/**
 * Validatorラッパー
 */
function initValidator(inputElement, errorFieldElement) {
  const name = inputElement.name;
  const schema = global.schemas[name];
  console.log(name, schema);
  if (!schema) {
    console.error(`No schema found for ${name}`);
    return;
  }
  inputElement.addEventListener("input", (event) => {
    console.log(event);
    const value = event.target.value;
    const result = validate(value, schema);
    if (result.valid) {
      event.target.classList.remove("invalid");
      if (errorFieldElement) {
        errorFieldElement.textContent = "";
      }
    } else {
      event.target.classList.add("invalid");
      if (errorFieldElement) {
        errorFieldElement.textContent = result.message;
      }
    }
  });
}

function validate(value, schema) {
  try {
    schema.parse(value);
    const result = {
      valid: true,
      message: "",
    };
    return result;
  } catch (e) {
    const errorMessage = error.errors[0]?.message || "入力が無効です";
    const result = {
      valid: false,
      message: errorMessage,
    };
    return result;
  }
}

document.addEventListener("DOMContentLoaded", (event) => {
  var elements = document.getElementsByClassName("validator-wrapper");
  for (const el of elements) {
    // validator-wrapperはdivタグなので、このdivタグの子要素のinputタグを取得する
    const inputElement = el.querySelector("input");
    // エラー表示フィールドがあれば、そこにバリデーションエラーメッセージを表示する
    const errorFieldElement = el.querySelector(".validation-error");
    if (!inputElement) {
      console.error("No input element found in validator-wrapper", el);
      continue;
    }
    initValidator(inputElement, errorFieldElement);
  }
});