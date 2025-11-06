// pip-video.js
(function (global) {
  function showPipVideo(srcObj, onClose = null) {
    // 如果已存在，先移除
    const old = document.getElementById("pip-container");
    if (old) old.remove();

    // 创建容器
    const pip = document.createElement("div");
    pip.id = "pip-container";
    pip.className = "box";
    pip.style.cssText = `
      position:fixed; bottom:2rem; right:2rem; width:320px; min-width:200px; min-height:180px; z-index:1000; padding:0;
      box-shadow:0 2px 8px rgba(0,0,0,0.2);
    `;

    // 标题栏
    const header = document.createElement("div");
    header.id = "pip-header";
    header.style.cssText = `
      cursor:move; background:#f5f5f5; padding:0.5rem; display:flex; align-items:center; justify-content:flex-end; border-bottom:1px solid #eee;
    `;
    header.innerHTML = `
      <button id="pip-close" class="button is-small is-white" style="border:none; box-shadow:none;" aria-label="关闭">
        <span class="icon">
          <svg width="20" height="20" viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
            <polyline points="7 13 13 7" />
            <polyline points="13 13 7 7" />
          </svg>
        </span>
      </button>
    `;

    // 内容区
    const content = document.createElement("div");
    content.id = "pip-content";
    content.style.cssText = "position:relative; width:100%; height:180px;";

    // 视频
    const video = document.createElement("video");
    video.id = "pip-video";
    // video.controls = true;
    video.autoplay = true;
    video.muted = true; // 静音，避免自动播放被阻止
    video.playsInline = true;
    video.style.cssText =
      "width:100%; height:100%; display:block; background:#000; border-radius:0 0 6px 6px;";

    // 缩放手柄
    const handle = document.createElement("span");
    handle.id = "pip-resize-handle";
    handle.className = "icon";
    handle.style.cssText =
      "position:absolute; right:8px; bottom:8px; cursor:se-resize; z-index:10;";
    handle.innerHTML = `
      <svg width="16" height="16" viewBox="0 0 16 16">
        <polyline points="4,12 12,12 12,4" fill="none" stroke="gray" stroke-width="2"/>
      </svg>
    `;

    content.appendChild(video);
    content.appendChild(handle);
    pip.appendChild(header);
    pip.appendChild(content);
    document.body.appendChild(pip);

    function getEventXY(e) {
      // 支持鼠标和触摸
      if (e.touches && e.touches.length) {
        return { x: e.touches[0].clientX, y: e.touches[0].clientY };
      } else if (e.changedTouches && e.changedTouches.length) {
        // 用于 touchend
        return {
          x: e.changedTouches[0].clientX,
          y: e.changedTouches[0].clientY,
        };
      } else {
        return { x: e.clientX, y: e.clientY };
      }
    }

    // 拖拽
    (function () {
      let offsetX = 0,
        offsetY = 0,
        dragging = false;

      function handleMouseDown(e) {
        if (e.target.closest("#pip-close")) return; // 不拖拽关闭按钮
        dragging = true;
        const rect = pip.getBoundingClientRect();
        const { x, y } = getEventXY(e);
        offsetX = x - rect.left;
        offsetY = y - rect.top;
        document.body.style.userSelect = "none";
        // 防止移动端页面滚动
        if (e.type.startsWith("touch")) e.preventDefault();
      }

      function handleMouseMove(e) {
        if (!dragging) return;
        const { x, y } = getEventXY(e);
        let newX = x - offsetX;
        let newY = y - offsetY;
        const winW = window.innerWidth,
          winH = window.innerHeight;
        const pipW = pip.offsetWidth,
          pipH = pip.offsetHeight;
        newX = Math.max(0, Math.min(newX, winW - pipW));
        newY = Math.max(0, Math.min(newY, winH - pipH));
        pip.style.left = newX + "px";
        pip.style.top = newY + "px";
        pip.style.right = "auto";
        pip.style.bottom = "auto";
        // 防止移动端页面滚动
        if (e.type.startsWith("touch")) e.preventDefault();
      }

      function handleMouseUp(e) {
        if(!dragging) return
        dragging = false;
        document.body.style.userSelect = "";
        // 防止移动端页面滚动
        if (e && e.type && e.type.startsWith("touch")) e.preventDefault();
      }
      pip.addEventListener("mousedown", handleMouseDown);
      document.addEventListener("mousemove", handleMouseMove);
      document.addEventListener("mouseup", handleMouseUp);

      pip.addEventListener("touchstart", handleMouseDown, { passive: false });
      document.addEventListener("touchmove", handleMouseMove, {
        passive: false,
      });
      document.addEventListener("touchend", handleMouseUp, { passive: false });
    })();

    // 缩放
    (function () {
      let resizing = false,
        startX = 0,
        startY = 0,
        startW = 0,
        startH = 0;
      const minW = 200,
        minH = 120,
        maxW = window.innerWidth,
        maxH = window.innerHeight;

      function handleResizeStart(e) {
        e.preventDefault();
        e.stopPropagation();
        resizing = true;
        const { x, y } = getEventXY(e);
        startX = x;
        startY = y;
        startW = pip.offsetWidth;
        startH = content.offsetHeight;
        document.body.style.userSelect = "none";
      }

      function handleResizeMove(e) {
        if (!resizing) return;
        const { x, y } = getEventXY(e);
        let newW = startW + (x - startX);
        let newH = startH + (y - startY);
        newW = Math.max(minW, Math.min(newW, maxW));
        newH = Math.max(minH, Math.min(newH, maxH));
        pip.style.width = newW + "px";
        content.style.height = newH + "px";
        if (e.type.startsWith("touch")) e.preventDefault();
      }

      function handleResizeEnd(e) {
        if(!resizing) return
        if (e.target.closest("#pip-close")) return; // 不拖拽关闭按钮
        resizing = false;
        document.body.style.userSelect = "";
        if (e && e.type && e.type.startsWith("touch")) e.preventDefault();
      }

      // 事件绑定
      handle.addEventListener("mousedown", handleResizeStart);
      handle.addEventListener("touchstart", handleResizeStart, {
        passive: false,
      });

      document.addEventListener("mousemove", handleResizeMove);
      document.addEventListener("touchmove", handleResizeMove, {
        passive: false,
      });

      document.addEventListener("mouseup", handleResizeEnd);
      document.addEventListener("touchend", handleResizeEnd, {
        passive: false,
      });
    })();

    // 关闭
    const onCloseCallback = function (e) {
      console.log(e);
      onClose && onClose();
      pip.remove();
    };
    header
      .querySelector("#pip-close")
      .addEventListener("click", onCloseCallback);
    // header.querySelector("#pip-close").addEventListener("touchend", onCloseCallback);
  }

  // 导出到全局
  global.showPipVideo = showPipVideo;
})(window);
