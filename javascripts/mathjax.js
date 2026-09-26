window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  },
  startup: {
    typeset: false,
    ready: () => {
      MathJax.startup.defaultReady();
      let pending = MathJax.startup.promise;
      const render = () => {
        pending = pending.then(() => {
          MathJax.startup.output.clearCache();
          MathJax.typesetClear();
          MathJax.texReset();
          return MathJax.typesetPromise();
        }).catch(error => console.error("公式渲染失败", error));
      };
      if (typeof document$ !== "undefined") {
        document$.subscribe(render);
      } else {
        render();
      }
    }
  }
};
