// ============================================================
// Go2 教材反馈按钮
// - 左下角悬浮,可临时关闭(关闭后当次会话不再出现)
// - 菜单三项:内容错误 / 代码错误 / 建议
// - 跳转 GitHub Issue Template,预填当前页面 URL
// ============================================================

(function () {
  "use strict";

  const REPO = "ztl3106742440-hub/go2-tutorial";
  const HIDE_KEY = "go2-feedback-hidden";

  const ITEMS = [
    {
      icon: "📝",
      label: "报告内容错误",
      template: "docs-bug.yml",
    },
    {
      icon: "💻",
      label: "报告代码错误",
      template: "code-bug.yml",
    },
    {
      icon: "💡",
      label: "提建议",
      template: "suggestion.yml",
    },
  ];

  function buildIssueUrl(template) {
    const pageUrl = encodeURIComponent(window.location.href);
    return (
      "https://github.com/" +
      REPO +
      "/issues/new?template=" +
      template +
      "&page-url=" +
      pageUrl
    );
  }

  function buildWidget() {
    if (document.getElementById("feedback-widget")) return;

    const widget = document.createElement("div");
    widget.id = "feedback-widget";
    widget.className = "feedback-widget";

    // 主按钮
    const btn = document.createElement("button");
    btn.className = "feedback-btn";
    btn.type = "button";
    btn.setAttribute("aria-label", "反馈问题");
    btn.innerHTML =
      '<span class="feedback-btn__icon">🐕</span>' +
      '<span class="feedback-btn__text">反馈问题</span>';

    // 关闭按钮
    const close = document.createElement("button");
    close.className = "feedback-close";
    close.type = "button";
    close.setAttribute("aria-label", "关闭反馈按钮");
    close.innerHTML = "&times;";

    // 菜单
    const menu = document.createElement("div");
    menu.className = "feedback-menu";
    ITEMS.forEach(function (item) {
      const a = document.createElement("a");
      a.className = "feedback-menu__item";
      a.href = buildIssueUrl(item.template);
      a.target = "_blank";
      a.rel = "noopener noreferrer";
      a.innerHTML =
        '<span class="feedback-menu__item-icon">' +
        item.icon +
        "</span><span>" +
        item.label +
        "</span>";
      // 每次点击都重新计算 URL,保证当前页面实时正确
      a.addEventListener("click", function (e) {
        a.href = buildIssueUrl(item.template);
      });
      menu.appendChild(a);
    });

    widget.appendChild(menu);
    widget.appendChild(btn);
    widget.appendChild(close);
    document.body.appendChild(widget);

    // 事件
    btn.addEventListener("click", function (e) {
      e.stopPropagation();
      menu.classList.toggle("feedback-menu--open");
    });

    close.addEventListener("click", function (e) {
      e.stopPropagation();
      widget.classList.add("feedback-widget--hidden");
      try {
        sessionStorage.setItem(HIDE_KEY, "1");
      } catch (err) {
        /* ignore */
      }
    });

    // 点菜单外关闭菜单(不隐藏按钮)
    document.addEventListener("click", function (e) {
      if (!widget.contains(e.target)) {
        menu.classList.remove("feedback-menu--open");
      }
    });

    // Esc 收菜单
    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape") {
        menu.classList.remove("feedback-menu--open");
      }
    });
  }

  function init() {
    try {
      if (sessionStorage.getItem(HIDE_KEY) === "1") return;
    } catch (err) {
      /* ignore */
    }
    buildWidget();
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
