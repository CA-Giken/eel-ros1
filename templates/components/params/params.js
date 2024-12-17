/**
 * ROS Parameters
 **/
document.addEventListener("DOMContentLoaded", async () => {
  /**
   * ROSPARAMの必須属性
   * - class="rosparam"
   * - name="[/param_name]"
   * - data-rtype="[PARAM_TYPES]"
   */
  const paramElements = document.getElementsByClassName("rosparam");
  for (const element of paramElements) {
    /**
     * HTMLタグバリデータ
     */
    console.assert(element.getAttribute("name") !== null, "Parameterのname属性が必要です.");
    const name = element.getAttribute("name");

    /**
     * ROSParamのポーリング登録
     */
    const event = new CustomEvent(ROS_EVENTS.Param, {
      detail: { name },
    });
    document.dispatchEvent(event);

    /**
     * DOM更新方法の登録
     */
    if (element.hasAttribute("update-custom")) {
      // 特殊なコールバックが必要な場合は、update-custom属性を設定する
      continue;
    }

    // rosparamのデータを受信した際のデフォルトDOM更新処理
    domUpdateHelper.registerCallback(element, domUpdateHelper.updateElement);


    /** ROSPARAM SETTER */
    if (element.tagName === "DIV") {
      const inputs = element.querySelectorAll("input");
      for (const input of inputs) {
        input.addEventListener("change", async (e) => {
          const value = e.currentTarget.value;
          const key = e.currentTarget.getAttribute("data-key");
          const obj = await ROSInterface.getParam(name);
          const event = new CustomEvent(ROS_EVENTS.ParamSet, {
            detail: {
              name,
              type: PARAM_TYPES.Object,
              value: Utils.EvalSetValue(obj, key, value)
            },
          });
          document.dispatchEvent(event);
        });
      }
    }

    if (element.tagName === "SELECT") {
      element.addEventListener("change", async (e) => {
        const value = e.currentTarget.value;
        const event = new CustomEvent(ROS_EVENTS.ParamSet, {
          detail: {
            name,
            type: PARAM_TYPES.Number,
            value
          },
        });
        document.dispatchEvent(event);
      });
    }

    if (element.tagName === "INPUT") {
      if (element.type === "checkbox") {
        element.addEventListener("change", async (e) => {
          const value = e.currentTarget.checked;
          const event = new CustomEvent(ROS_EVENTS.ParamSet, {
            detail: {
              name,
              type: PARAM_TYPES.Bool,
              value
            },
          });
          document.dispatchEvent(event);
        });
      }
      else if (element.type === "radio") {
        element.addEventListener("change", async (e) => {
          const value = e.currentTarget.value;
          const event = new CustomEvent(ROS_EVENTS.ParamSet, {
            detail: {
              name,
              type: PARAM_TYPES.Number,
              value
            },
          });
          document.dispatchEvent(event);
        });
      }
      else if (element.type === "text") {
        element.addEventListener("change", async (e) => {
          const value = e.currentTarget.value;
          const event = new CustomEvent(ROS_EVENTS.ParamSet, {
            detail: {
              name,
              type: PARAM_TYPES.String,
              value
            },
          });
          document.dispatchEvent(event);
        });
      } else {
        element.addEventListener("change", async (e) => {
          const value = e.currentTarget.value;
          const event = new CustomEvent(ROS_EVENTS.ParamSet, {
            detail: {
              name,
              type: PARAM_TYPES.Number,
              value
            },
          });
          document.dispatchEvent(event);
        });
      }
    }
  }

  /**
   * DOM更新イベントの登録
   */
  document.addEventListener(ROS_EVENTS.ParamUpdated, async (e) => {
    const { name, type, value } = e.detail;
    const paramElements = document.getElementsByName(name);
    paramElements.forEach((element) => {
      domUpdateHelper.executeCallbacks(element, { type, value });
    });
  });
});