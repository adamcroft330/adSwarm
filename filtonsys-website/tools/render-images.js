// Renders the raster images the pages reference: og.png (social share card),
// apple-touch-icon.png and logo.png. Run after `python3 build.py`:
//
//   npm i -D playwright && npx playwright install chromium   (once)
//   node tools/render-images.js
//
const path = require("path");
const { chromium } = require("playwright");

const fs = require("fs");

const pub = path.resolve(__dirname, "..", "public");
const img = path.join(pub, "assets", "img");

// Pages rendered with setContent cannot fetch file:// URLs, so inline assets.
const dataUri = (rel, type) =>
  `data:${type};base64,${fs.readFileSync(path.join(pub, rel)).toString("base64")}`;
const FLOW = dataUri("assets/img/flow.svg", "image/svg+xml");
const MARK = dataUri("assets/img/favicon.svg", "image/svg+xml");

const fonts = `
@font-face { font-family: Inter; src: url(${dataUri("assets/fonts/inter-latin-wght.woff2", "font/woff2")}); font-weight: 100 900; }
@font-face { font-family: Mono; src: url(${dataUri("assets/fonts/jetbrains-mono-latin-500.woff2", "font/woff2")}); }
* { margin: 0; box-sizing: border-box; }`;

const og = `<!doctype html><html><head><style>${fonts}
body { width: 1200px; height: 630px; background: #0b1a29; color: #fff; font-family: Inter; position: relative; overflow: hidden; }
.art { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: cover; object-position: 75% 50%; }
.shade { position: absolute; inset: 0; background: linear-gradient(90deg, rgba(11,26,41,.97) 0%, rgba(11,26,41,.85) 45%, rgba(11,26,41,.2) 100%); }
.in { position: absolute; inset: 72px 80px; display: flex; flex-direction: column; }
.brand { display: flex; align-items: center; gap: 16px; }
.brand img { width: 52px; height: 52px; }
.brand b { display: block; font-size: 26px; font-weight: 700; }
.brand span { display: block; font-family: Mono; font-size: 13px; letter-spacing: .18em; text-transform: uppercase; color: #9db0c3; margin-top: 4px; }
h1 { margin-top: auto; font-size: 64px; line-height: 1.06; letter-spacing: -.035em; font-weight: 650; max-width: 15ch; }
p { margin-top: 24px; font-family: Mono; font-size: 17px; letter-spacing: .12em; text-transform: uppercase; color: #7fdcf2; }
</style></head><body>
<img class="art" src="${FLOW}"><div class="shade"></div>
<div class="in">
  <div class="brand"><img src="${MARK}"><div><b>Element</b><span>Digital Engineering</span></div></div>
  <h1>Fluid systems, engineered from concept to certification.</h1>
  <p>Formerly Filton Systems Engineering &middot; Bristol, UK</p>
</div></body></html>`;

const icon = (size, pad) => `<!doctype html><html><head><style>${fonts}
body { width: ${size}px; height: ${size}px; background: #0b1a29; display: grid; place-items: center; }
img { width: ${size - pad * 2}px; height: ${size - pad * 2}px; }
</style></head><body><img src="${MARK}"></body></html>`;

(async () => {
  const browser = await chromium.launch();
  const shots = [
    ["og.png", og, 1200, 630],
    ["apple-touch-icon.png", icon(180, 0), 180, 180],
    ["logo.png", icon(512, 0), 512, 512],
  ];
  for (const [name, html, w, h] of shots) {
    const page = await browser.newPage({ viewport: { width: w, height: h } });
    await page.setContent(html, { waitUntil: "networkidle" });
    await page.evaluate(() => document.fonts.ready);
    await page.waitForTimeout(300);
    await page.screenshot({ path: path.join(img, name), animations: "disabled" });
    await page.close();
    console.log("wrote", path.join("public/assets/img", name));
  }
  await browser.close();
})();
