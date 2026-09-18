// 使用 Archify 自带的浏览器导出，保留字体、深浅主题及原始拓扑。
import fs from 'node:fs/promises';
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const [archifyRoot, input, output] = process.argv.slice(2);
if (!archifyRoot || !input || !output) {
  throw new Error('用法: node Export_Svg.mjs <Archify目录> <输入HTML> <输出SVG>');
}

const { ChromeVisualBrowser, findChrome } = await import(
  pathToFileURL(path.resolve(archifyRoot, 'bin/visual-check.mjs')).href
);
const chrome = findChrome();
if (!chrome) throw new Error('导出 SVG 需要 Chrome / Chromium，可通过 ARCHIFY_CHROME 指定。');

const browser = new ChromeVisualBrowser(chrome);
try {
  const session = await browser.sessionPromise;
  const send = (method, params = {}) => browser.cdp.send(method, params, session);
  const loaded = browser.cdp.waitFor('Page.loadEventFired', session);
  await send('Page.navigate', { url: pathToFileURL(path.resolve(input)).href });
  await loaded;

  const result = await send('Runtime.evaluate', {
    awaitPromise: true,
    returnByValue: true,
    expression: `(async () => {
      await document.fonts.ready;
      let exported;
      const createUrl = URL.createObjectURL;
      const click = HTMLAnchorElement.prototype.click;
      URL.createObjectURL = blob => {
        if (blob.type.startsWith('image/svg+xml')) exported = blob;
        return createUrl.call(URL, blob);
      };
      HTMLAnchorElement.prototype.click = function () {
        if (!this.download) click.call(this);
      };
      try {
        await Archify.exportMenu.run('svg');
        if (!exported) throw new Error('Archify 未生成 SVG');
        return await exported.text();
      } finally {
        URL.createObjectURL = createUrl;
        HTMLAnchorElement.prototype.click = click;
      }
    })()`
  });
  if (result.exceptionDetails) {
    throw new Error(result.exceptionDetails.exception?.description || 'SVG 导出失败');
  }
  await fs.writeFile(path.resolve(output), result.result.value, 'utf8');
  console.log(`SVG 已导出: ${path.resolve(output)}`);
} finally {
  await browser.close();
}
