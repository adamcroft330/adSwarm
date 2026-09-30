#!/usr/bin/env python3
"""Build the static site into ./public.

    python3 build.py

No dependencies beyond the Python 3 standard library. Copy lives in
content.py; this file holds the page templates. Every page is written as
<path>/index.html so the old site's URLs (e.g. /what-we-do/surge-analysis)
keep working. Links are relative, so the output also works from a sub-folder.
"""

import datetime
import hashlib
import html
import json
import math
from pathlib import Path

import content as C
import flowfield

HERE = Path(__file__).resolve().parent
OUT = HERE / "public"
SITE = C.SITE
TODAY = datetime.date.today().isoformat()
BY_SLUG = {s["slug"]: s for s in C.SERVICES}
QUALITY_PATH = "en9100-quality-certificate-and-cyber-essentials-plus"


def esc(s):
    return html.escape(s, quote=True)


# ---------------------------------------------------------------------------
# Icons (24x24, stroked)
# ---------------------------------------------------------------------------
ICONS = {
    "mesh": '<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18" opacity=".4"/><path d="M3 18.5C9 18 11 7 21 5.5"/>',
    "surge": '<path d="M2 15h4l2.2-10 3.1 15 2.4-10 1.9 6 1.5-3H22"/>',
    "charge": '<path d="M13 2 4.5 13.5H11L10 22l8.5-11.5H12L13 2z"/>',
    "ice": '<path d="M12 2v20M3.4 7l17.2 10M20.6 7 3.4 17"/><path d="m9.5 3.5 2.5 2 2.5-2M9.5 20.5l2.5-2 2.5 2M4 10.5l2.5-1-.5-2.7M20 13.5l-2.5 1 .5 2.7M4 13.5l2.5 1-.5 2.7M20 10.5l-2.5-1 .5-2.7"/>',
    "valve": '<path d="M3 8v10l9-5-9-5zM21 8v10l-9-5 9-5z"/><path d="M12 13V5M8.5 5h7"/>',
    "shield": '<path d="M12 3 4.5 6v6c0 4.6 3.2 7.8 7.5 9 4.3-1.2 7.5-4.4 7.5-9V6L12 3z"/><path d="m8.7 12.2 2.3 2.3 4.4-4.6"/>',
    "gauge": '<path d="M3.5 17.5a9 9 0 1 1 17 0"/><path d="m12 14 4.2-5.2"/><circle cx="12" cy="14" r="1.6"/><path d="M6.3 11.5l1.2.6M12 6.5V8M17.7 11.5l-1.2.6" opacity=".6"/>',
    "doc": '<path d="M14 3H6.5A1.5 1.5 0 0 0 5 4.5v15A1.5 1.5 0 0 0 6.5 21h11a1.5 1.5 0 0 0 1.5-1.5V8l-5-5z"/><path d="M14 3v5h5M8.5 12.5h7M8.5 16.5h5"/>',
    "plane": '<path d="M21 15.5v-1.7l-8-5V4a1.5 1.5 0 0 0-3 0v4.8l-8 5v1.7l8-2.4V18l-2.2 1.6V21l3.7-1 3.7 1v-1.4L13 18v-4.9l8 2.4z"/>',
    "h2": '<circle cx="7" cy="12" r="4"/><circle cx="17" cy="12" r="4"/><path d="M11 12h2"/><path d="M7 10.5v3M17 10.5v3" opacity=".6"/>',
    "wind": '<path d="M12 10.5V22M9 22h6"/><circle cx="12" cy="9" r="1.5"/><path d="M12 7.5c-.6-2.2-.4-4 .2-5.5 1.3 1.6 1.5 3.6-.2 5.5zM13.3 9.8c2.2.6 3.6 1.8 4.4 3.3-2 .3-3.8-.6-4.4-3.3zM10.7 9.8c-1.6 1.6-3.4 2.2-5 2.2.8-1.9 2.5-2.8 5-2.2z"/>',
    "pipe": '<path d="M2 9.5h5v5H2M22 9.5h-5v5h5M7 11h10M7 13h10"/><path d="M7 7.5v9M17 7.5v9"/>',
    "drop": '<path d="M12 3s6.5 6.6 6.5 11.2a6.5 6.5 0 0 1-13 0C5.5 9.6 12 3 12 3z"/><path d="M9 14.5a3 3 0 0 0 3 3" opacity=".6"/>',
    "atom": '<circle cx="12" cy="12" r="1.6"/><ellipse cx="12" cy="12" rx="10" ry="4"/><ellipse cx="12" cy="12" rx="10" ry="4" transform="rotate(60 12 12)"/><ellipse cx="12" cy="12" rx="10" ry="4" transform="rotate(120 12 12)"/>',
    "arrow": '<path d="M5 12h14M13 6l6 6-6 6"/>',
    "check": '<path d="m5 12.5 4.5 4.5L19 7.5"/>',
    "mail": '<rect x="3" y="5" width="18" height="14" rx="2"/><path d="m3.5 6.5 8.5 7 8.5-7"/>',
    "phone": '<path d="M5.2 3.5h3.3l1.8 4.6-2.3 1.4a11.5 11.5 0 0 0 5.5 5.5l1.4-2.3 4.6 1.8v3.3A2 2 0 0 1 17.4 20 15 15 0 0 1 3.2 5.6a2 2 0 0 1 2-2.1z"/>',
    "pin": '<path d="M12 21s-7-6.1-7-11.4a7 7 0 0 1 14 0C19 14.9 12 21 12 21z"/><circle cx="12" cy="9.6" r="2.5"/>',
    "chevron": '<path d="m6 9 6 6 6-6"/>',
    "menu": '<path d="M4 7h16M4 12h16M4 17h16"/>',
    "close": '<path d="M6 6l12 12M18 6 6 18"/>',
    "download": '<path d="M12 4v11M7 10l5 5 5-5M5 20h14"/>',
    "external": '<path d="M14 4h6v6M20 4l-9 9M18 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V7a1 1 0 0 1 1-1h5"/>',
    "award": '<circle cx="12" cy="9" r="6"/><path d="m8.5 14 -1.5 7 5-2.5 5 2.5-1.5-7"/><path d="m9.6 9 1.6 1.6 3.2-3.2"/>',
    "lock": '<rect x="4.5" y="10.5" width="15" height="10.5" rx="2"/><path d="M8 10.5V7.5a4 4 0 0 1 8 0v3"/><circle cx="12" cy="15.5" r="1.4"/>',
}


def icon(name, cls="icon"):
    return ('<svg class="{}" viewBox="0 0 24 24" aria-hidden="true" focusable="false">{}</svg>'
            .format(cls, ICONS[name]))


MARK_INNER = (
    '<rect width="32" height="32" rx="8" fill="#0B1A29"/>'
    '<path d="M5 12.6c4.8-3.3 9.6-3.8 13.3-2.6 3.1 1 5.8 1.2 8.7.3" stroke="#3CCFEF" stroke-width="2.2" fill="none" stroke-linecap="round"/>'
    '<path d="M5 17.6c4.8-1.7 8.8-1.3 12.5-.3 3.4.9 6.5 1 9.5.4" stroke="#7FDCF2" stroke-opacity=".75" stroke-width="2" fill="none" stroke-linecap="round"/>'
    '<path d="M5 22.6c6-.9 11.2-.4 22 .6" stroke="#7FDCF2" stroke-opacity=".4" stroke-width="1.8" fill="none" stroke-linecap="round"/>'
)
MARK = '<svg class="brand-mark" viewBox="0 0 32 32" aria-hidden="true" focusable="false">{}</svg>'.format(MARK_INNER)


# ---------------------------------------------------------------------------
# Small illustrations for the three specialisms
# ---------------------------------------------------------------------------
def art_surge():
    pts = [(0, 62), (44, 62)]
    for i in range(0, 196, 2):
        t = float(i)
        y = 62 - 52 * math.exp(-t / 46) * math.cos(2 * math.pi * t / 30)
        pts.append((46 + t, y))
    d = "M" + " L".join("{:.1f} {:.1f}".format(x, y) for x, y in pts)
    return (
        '<svg viewBox="0 0 244 84" aria-hidden="true" focusable="false" fill="none" stroke="currentColor">'
        '<path d="M0 62H244" stroke-opacity=".3" stroke-dasharray="3 5"/>'
        '<path d="M0 10H244" stroke-opacity=".3" stroke-dasharray="3 5"/>'
        '<path d="{}" stroke-width="2" stroke-linejoin="round"/>'
        '<text x="244" y="76" text-anchor="end" fill="currentColor" stroke="none" font-family="JetBrains Mono, monospace" font-size="9" opacity=".7">P op</text>'
        '<text x="244" y="24" text-anchor="end" fill="currentColor" stroke="none" font-family="JetBrains Mono, monospace" font-size="9" opacity=".7">P peak</text>'
        "</svg>".format(d)
    )


def art_charge():
    parts = [
        '<svg viewBox="0 0 244 84" aria-hidden="true" focusable="false" fill="none" stroke="currentColor">',
        '<path d="M0 14H244M0 70H244" stroke-width="2"/>',
    ]
    for x in range(18, 244, 34):  # charge on the wall
        parts.append('<path d="M{0} 5v6M{1} 8h6" stroke-opacity=".75" stroke-width="1.5"/>'.format(x, x - 3))
        parts.append('<path d="M{0} 73v6M{1} 76h6" stroke-opacity=".75" stroke-width="1.5"/>'.format(x + 17, x + 14))
    for i, x in enumerate(range(30, 230, 26)):  # opposite charge carried in the flow
        y = 30 + (i * 17) % 26
        parts.append('<circle cx="{}" cy="{}" r="6" stroke-opacity=".6"/><path d="M{} {}h6" stroke-width="1.5"/>'.format(x, y, x - 3, y))
    parts.append('<path d="M92 56h60M144 50l8 6-8 6" stroke-opacity=".5"/>')
    parts.append("</svg>")
    return "".join(parts)


def art_ice():
    def flake(cx, cy, r):
        segs = []
        for k in range(3):
            a = math.pi / 3 * k + math.pi / 6
            dx, dy = r * math.cos(a), r * math.sin(a)
            segs.append("M{:.1f} {:.1f}L{:.1f} {:.1f}".format(cx - dx, cy - dy, cx + dx, cy + dy))
        return '<path d="{}" stroke-width="1.5"/>'.format("".join(segs))

    # ice accretion building along the lower wall, heaviest downstream
    acc = ["M0 70"]
    for i in range(0, 245, 8):
        h = 2 + 20 * (i / 244) ** 1.6 * (0.7 + 0.3 * math.sin(i * 0.9))
        acc.append("L{} {:.1f}".format(i, 70 - h))
    acc.append("L244 70Z")
    return (
        '<svg viewBox="0 0 244 84" aria-hidden="true" focusable="false" fill="none" stroke="currentColor">'
        '<path d="M0 14H244M0 70H244" stroke-width="2"/>'
        '<path d="{}" fill="currentColor" fill-opacity=".18" stroke-opacity=".7" stroke-linejoin="round"/>'
        '{}{}{}'
        '<circle cx="36" cy="34" r="2.5" fill="currentColor" stroke="none" opacity=".6"/>'
        '<circle cx="64" cy="44" r="2" fill="currentColor" stroke="none" opacity=".6"/>'
        '<circle cx="96" cy="30" r="2.5" fill="currentColor" stroke="none" opacity=".6"/>'
        "</svg>".format(" ".join(acc), flake(140, 34, 8), flake(184, 28, 6), flake(216, 40, 7))
    )


SPEC_ART = {"surge-analysis": art_surge, "electro-statics": art_charge, "ice-and-water-in-fuel": art_ice}


# ---------------------------------------------------------------------------
# Shared chrome
# ---------------------------------------------------------------------------
def asset_hash(rel):
    return hashlib.sha1((OUT / rel).read_bytes()).hexdigest()[:10]


def header(root, active):
    sub_items = "".join(
        '<li><a href="{r}what-we-do/{slug}/">{ic}<span><strong>{t}</strong><span>{s}</span></span></a></li>'.format(
            r=root, slug=s["slug"], ic=icon(s["icon"]), t=esc(s["title"]), s=esc(s["summary"])
        )
        for s in C.SERVICES
    )

    def link(key, href, label):
        cur = ' aria-current="page"' if active == key else ""
        return '<li><a class="nav-link" href="{}"{}>{}</a></li>'.format(href, cur, label)

    wwd_cls = "nav-link is-active" if active and active.startswith("what-we-do") else "nav-link"
    wwd_cur = ' aria-current="page"' if active == "what-we-do" else ""
    return """<header class="site-header" data-header>
  <div class="container header-inner">
    <a class="brand" href="{root_home}">{mark}<span class="brand-text"><strong>Element</strong><span>Digital Engineering</span></span><span class="sr-only"> home</span></a>
    <nav class="nav" aria-label="Main">
      <button class="nav-toggle" type="button" aria-expanded="false" aria-controls="nav-menu"><span class="sr-only">Open menu</span>{menu}{close}</button>
      <div class="nav-menu" id="nav-menu">
        <ul class="nav-list">
          <li class="has-sub">
            <a class="{wwd_cls}" href="{root}what-we-do/"{wwd_cur}>What we do</a>
            <button class="sub-toggle" type="button" aria-expanded="false" aria-controls="sub-capabilities"><span class="sr-only">Show capabilities</span>{chev}</button>
            <div class="sub" id="sub-capabilities">
              <ul class="sub-grid">{sub_items}</ul>
              <div class="sub-foot"><span class="muted">Fluid systems engineering from concept to certification.</span><a href="{root}what-we-do/">All capabilities</a></div>
            </div>
          </li>
          {hydrogen}
          {about}
          {quality}
        </ul>
        <a class="btn btn-primary nav-cta" href="{root}contact/"{contact_cur}>Contact us</a>
      </div>
    </nav>
  </div>
</header>""".format(
        root=root,
        root_home=root or "./",
        mark=MARK,
        menu=icon("menu", "icon icon-menu"),
        close=icon("close", "icon icon-close"),
        chev=icon("chevron"),
        wwd_cls=wwd_cls,
        wwd_cur=wwd_cur,
        sub_items=sub_items,
        hydrogen=link("hydrogen", root + "hydrogen/", "Hydrogen"),
        about=link("about", root + "about/", "About"),
        quality=link("quality", root + QUALITY_PATH + "/", "Quality"),
        contact_cur=' aria-current="page"' if active == "contact" else "",
    )


def footer(root):
    caps = "".join('<li><a href="{}what-we-do/{}/">{}</a></li>'.format(root, s["slug"], esc(s["title"])) for s in C.SERVICES)
    addr = "<br>".join(esc(a) for a in SITE["address_lines"])
    return """<footer class="site-footer on-dark">
  <div class="container">
    <div class="footer-grid">
      <div class="footer-about">
        <a class="brand" href="{root_home}">{mark}<span class="brand-text"><strong>Element</strong><span>Digital Engineering</span></span><span class="sr-only"> home</span></a>
        <p>Fluid systems engineering from concept to certification. Formerly {former}.</p>
      </div>
      <nav aria-labelledby="footer-caps"><h2 id="footer-caps">Capabilities</h2><ul>{caps}</ul></nav>
      <nav aria-labelledby="footer-company"><h2 id="footer-company">Company</h2><ul>
        <li><a href="{root}about/">About us</a></li>
        <li><a href="{root}hydrogen/">Hydrogen</a></li>
        <li><a href="{root}{quality}/">Quality and certification</a></li>
        <li><a href="{root}contact/">Contact</a></li>
        <li><a href="https://www.element.com" rel="noopener">Element Materials Technology</a></li>
      </ul></nav>
      <div><h2>Get in touch</h2><ul class="footer-contact">
        <li>{pin}<address>{addr}</address></li>
        <li>{phone}<a href="tel:{tel}">{phone_d}</a></li>
        <li>{mail}<a href="mailto:{email}">{email}</a></li>
      </ul></div>
    </div>
    <div class="footer-bottom">
      <p>&copy; <span data-year>{year}</span> {legal}, part of Element. Registered in England and Wales, company no. {co}.</p>
      <p>EN 9100:2018 &middot; ISO 9001:2015 &middot; Cyber Essentials Plus</p>
    </div>
  </div>
</footer>""".format(
        root=root,
        root_home=root or "./",
        mark=MARK,
        former=esc(SITE["former_name"]),
        caps=caps,
        quality=QUALITY_PATH,
        pin=icon("pin"),
        phone=icon("phone"),
        mail=icon("mail"),
        addr=addr,
        tel=SITE["phone_href"],
        phone_d=esc(SITE["phone_display"]),
        email=SITE["email"],
        year=datetime.date.today().year,
        legal=esc(SITE["legal"]),
        co=SITE["company_no"],
    )


def org_jsonld():
    a = SITE["address_lines"]
    return {
        "@context": "https://schema.org",
        "@type": "Organization",
        "name": SITE["name"],
        "alternateName": SITE["former_name"],
        "legalName": SITE["legal"],
        "url": SITE["url"] + "/",
        "logo": SITE["url"] + "/assets/img/logo.png",
        "email": SITE["email"],
        "telephone": SITE["phone_href"],
        "foundingDate": "2012",
        "address": {
            "@type": "PostalAddress",
            "streetAddress": "{}, {}".format(a[0], a[1]),
            "addressLocality": a[2],
            "postalCode": a[3],
            "addressCountry": "GB",
        },
        "parentOrganization": {"@type": "Organization", "name": "Element Materials Technology", "url": "https://www.element.com"},
    }


def crumbs_jsonld(crumbs):
    items = []
    for i, (label, path) in enumerate(crumbs, 1):
        items.append({"@type": "ListItem", "position": i, "name": label,
                      "item": SITE["url"] + "/" + (path + "/" if path else "")})
    return {"@context": "https://schema.org", "@type": "BreadcrumbList", "itemListElement": items}


PAGES = []  # (path) for the sitemap


def write_page(path, title, description, main, active=None, jsonld=None, root=None, index=True):
    """path '' is the home page; '404.html' is written as-is with root-absolute links."""
    if root is None:
        depth = len([p for p in path.split("/") if p])
        root = "../" * depth
    if path.endswith(".html"):
        canonical = SITE["url"] + "/" + path
    else:
        canonical = SITE["url"] + "/" + (path + "/" if path else "")
    css_v, js_v = asset_hash("assets/css/styles.css"), asset_hash("assets/js/main.js")
    ld = ""
    for block in jsonld or []:
        ld += '<script type="application/ld+json">{}</script>\n'.format(json.dumps(block, ensure_ascii=False))
    doc = """<!doctype html>
<html lang="en-GB">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{title}</title>
<meta name="description" content="{desc}">
{robots}<link rel="canonical" href="{canonical}">
<meta name="theme-color" content="#0b1a29">
<meta property="og:type" content="website">
<meta property="og:site_name" content="{site}">
<meta property="og:title" content="{title}">
<meta property="og:description" content="{desc}">
<meta property="og:url" content="{canonical}">
<meta property="og:image" content="{url}/assets/img/og.png">
<meta property="og:locale" content="en_GB">
<meta name="twitter:card" content="summary_large_image">
<link rel="icon" href="{root}assets/img/favicon.svg" type="image/svg+xml">
<link rel="apple-touch-icon" href="{root}assets/img/apple-touch-icon.png">
<link rel="preload" href="{root}assets/fonts/inter-latin-wght.woff2" as="font" type="font/woff2" crossorigin>
<link rel="stylesheet" href="{root}assets/css/styles.css?v={css_v}">
<script>document.documentElement.classList.add("js")</script>
<script src="{root}assets/js/main.js?v={js_v}" defer></script>
{ld}</head>
<body>
<a class="skip" href="#main">Skip to content</a>
{header}
<main id="main">
{main}
</main>
{footer}
</body>
</html>
""".format(
        title=esc(title), desc=esc(description), canonical=canonical, site=esc(SITE["name"]), url=SITE["url"],
        robots='<meta name="robots" content="noindex">\n' if not index else "",
        root=root, css_v=css_v, js_v=js_v, ld=ld,
        header=header(root, active), main=main, footer=footer(root),
    )
    if path.endswith(".html"):
        target = OUT / path
    else:
        target = OUT / path / "index.html" if path else OUT / "index.html"
        PAGES.append(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(doc, encoding="utf-8")


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------
def page_hero(root, title, lead, eyebrow=None, crumbs=None, icon_name=None, actions="", extra=""):
    crumb_html = ""
    if crumbs:
        items = []
        for i, (label, path) in enumerate(crumbs):
            if i == len(crumbs) - 1:
                items.append('<li><span aria-current="page">{}</span></li>'.format(esc(label)))
            else:
                items.append('<li><a href="{}{}">{}</a></li>'.format(root, path + "/" if path else "", esc(label)))
        crumb_html = '<nav aria-label="Breadcrumb"><ol class="crumbs">{}</ol></nav>'.format("".join(items))
    return """<section class="hero hero--page on-dark">
  <img class="hero-art" src="{root}assets/img/flow.svg" alt="" width="1600" height="900">
  <div class="hero-shade"></div>
  <div class="container">
    {crumbs}
    {icon}{eyebrow}
    <h1>{title}</h1>
    <p class="lead">{lead}</p>
    {actions}
    {extra}
  </div>
</section>""".format(
        root=root, crumbs=crumb_html,
        icon='<div class="hero-icon">{}</div>'.format(icon(icon_name)) if icon_name else "",
        eyebrow='<p class="eyebrow">{}</p>'.format(esc(eyebrow)) if eyebrow else "",
        title=title, lead=lead, actions=actions, extra=extra,
    )


def stats_block():
    return '<dl class="stats">{}</dl>'.format("".join(
        "<div><dt>{}</dt><dd>{}</dd></div>".format(esc(v), esc(k)) for v, k in C.STATS))


def service_card(root, s):
    return """<li><a class="card" href="{root}what-we-do/{slug}/">
  <span class="card-icon">{icon}</span>
  <h3>{title}</h3>
  <p>{summary}</p>
  <span class="card-more">Learn more {arrow}</span>
</a></li>""".format(root=root, slug=s["slug"], icon=icon(s["icon"]), title=esc(s["title"]),
                    summary=esc(s["summary"]), arrow=icon("arrow"))


def lifecycle():
    return '<ol class="lifecycle" data-reveal-group>{}</ol>'.format("".join(
        "<li><small>{:02d}</small><strong>{}</strong><span>{}</span></li>".format(n, esc(a), esc(b))
        for n, (a, b) in enumerate(C.LIFECYCLE, 1)))


def chips(items):
    return '<ul class="chips">{}</ul>'.format("".join("<li>{}</li>".format(esc(i)) for i in items))


def checklist(items):
    return '<ul class="checklist">{}</ul>'.format("".join(
        "<li>{}<span>{}</span></li>".format(icon("check"), esc(i)) for i in items))


def cta(root, title="Have a fluid systems challenge?",
        text="Tell us about your system or programme and we will put you in touch with the right specialist."):
    return """<section class="section section--tight">
  <div class="container">
    <div class="cta on-dark" data-reveal>
      <div><h2>{title}</h2><p>{text}</p></div>
      <div class="actions">
        <a class="btn btn-primary" href="{root}contact/">Contact us {arrow}</a>
        <a class="btn btn-ghost" href="mailto:{email}">{email}</a>
      </div>
    </div>
  </div>
</section>""".format(root=root, title=title, text=text, arrow=icon("arrow"), email=SITE["email"])


# ---------------------------------------------------------------------------
# Pages
# ---------------------------------------------------------------------------
def build_home():
    r = ""
    cards = "".join(service_card(r, s) for s in C.SERVICES)
    specs = []
    for n, (slug, title, text) in enumerate(C.SPECIALISMS, 1):
        specs.append("""<a class="spec" href="what-we-do/{slug}/">
  <span class="spec-num">0{n}</span>
  <div class="spec-art">{art}</div>
  <h3>{title}</h3>
  <p>{text}</p>
  <span class="link-arrow">Learn more {arrow}</span>
</a>""".format(slug=slug, n=n, art=SPEC_ART[slug](), title=esc(title), text=esc(text), arrow=icon("arrow")))

    industries = []
    for item in C.INDUSTRIES:
        ic, title, text = item[:3]
        href = item[3] if len(item) > 3 else None
        inner = "{}<div><h3>{}{}</h3><p>{}</p></div>".format(
            icon(ic), esc(title), icon("arrow") if href else "", esc(text))
        if href:
            industries.append('<li><a class="industry" href="{}">{}</a></li>'.format(href, inner))
        else:
            industries.append('<li><div class="industry">{}</div></li>'.format(inner))

    h = C.HYDROGEN
    main = """
<section class="hero hero--home on-dark">
  <img class="hero-art" src="assets/img/flow.svg" alt="" width="1600" height="900" fetchpriority="high">
  <div class="hero-shade"></div>
  <div class="container hero-inner">
    <p class="eyebrow">Fluid systems engineering &middot; Bristol, UK</p>
    <h1>Fluid systems, engineered from concept to certification.</h1>
    <p class="lead">We design, model, test and certify fuel, air, hydraulic, inerting and engine systems for aerospace, and bring the same rigour to hydrogen, energy, water and nuclear.</p>
    <div class="actions">
      <a class="btn btn-primary" href="what-we-do/">Explore capabilities {arrow}</a>
      <a class="btn btn-ghost" href="contact/">Talk to an engineer</a>
    </div>
  </div>
  <div class="container">{stats}</div>
</section>

<section class="section">
  <div class="container split">
    <div data-reveal>
      <p class="eyebrow">Who we are</p>
      <h2>A single source for fluid systems.</h2>
    </div>
    <div class="prose lead" data-reveal>
      <p>Element Digital Engineering, formerly Filton Systems Engineering, provides fluid system engineering to a wide range of industries across the globe. We deliver solutions for the successful development of fluid systems and equipment, from concept to manufacture.</p>
      <p>Originating in the aerospace industry, we have a recognised capability in designing fuel, air, hydraulic, inerting and engine systems, with many of our engineers holding 10 to 25 years' experience.</p>
      {systems}
    </div>
  </div>
  <div class="container">{lifecycle}</div>
</section>

<section class="section section--tint" id="capabilities">
  <div class="container">
    <div class="section-head" data-reveal>
      <div>
        <p class="eyebrow">What we do</p>
        <h2>Capabilities</h2>
        <p>Mechanical design, simulation, analysis, prototyping, technical writing and testing, under one roof.</p>
      </div>
      <a class="link-arrow" href="what-we-do/">All capabilities {arrow}</a>
    </div>
    <ul class="card-grid" data-reveal-group>{cards}</ul>
  </div>
</section>

<section class="section section--dark on-dark">
  <div class="container">
    <div class="section-head" data-reveal>
      <div>
        <p class="eyebrow">Where we are known</p>
        <h2>The hard problems in fluid systems.</h2>
        <p>Three specialisms where our engineers are recognised experts, and where customers bring their most demanding challenges.</p>
      </div>
    </div>
    <div class="spec-grid" data-reveal-group>{specs}</div>
  </div>
</section>

<section class="section">
  <div class="container">
    <div class="section-head" data-reveal>
      <div>
        <p class="eyebrow">Industries</p>
        <h2>Born in aerospace. Applied wherever fluids flow.</h2>
        <p>We have expanded to offer our expertise to a range of industries reliant on fluid systems.</p>
      </div>
    </div>
    <ul class="industry-grid" data-reveal-group>{industries}</ul>
  </div>
</section>

<section class="section section--tight" style="padding-top:0">
  <div class="container">
    <div class="feature on-dark" data-reveal>
      <div class="feature-body">
        <p class="eyebrow">Hydrogen</p>
        <h2>Engineering liquid hydrogen for zero-emission flight.</h2>
        <p>{h_body}</p>
        <div class="actions">
          <a class="btn btn-primary" href="hydrogen/">Our hydrogen capability {arrow}</a>
        </div>
      </div>
      <div class="feature-art">
        <img src="assets/img/flow.svg" alt="" width="1600" height="900" loading="lazy">
        <div class="feature-facts">{facts}</div>
      </div>
    </div>
  </div>
</section>

<section class="section section--tint section--tight">
  <div class="container">
    <div class="section-head" data-reveal>
      <div>
        <p class="eyebrow">Quality</p>
        <h2>Certified to aerospace standards.</h2>
      </div>
      <a class="link-arrow" href="{quality}/">Quality and certification {arrow}</a>
    </div>
    <ul class="badges" data-reveal-group>{badges}</ul>
  </div>
</section>

{cta}
""".format(
        arrow=icon("arrow"),
        stats=stats_block(),
        systems=chips(C.SYSTEMS),
        lifecycle=lifecycle(),
        cards=cards,
        specs="".join(specs),
        industries="".join(industries),
        h_body=h["body"][1],
        facts="".join('<div class="fact"><strong>{}</strong><span>{}</span></div>'.format(a, esc(b))
                      for a, b in [("Kemble, UK", "Dedicated hydrogen facility with in-house liquefaction plant")] + h["project"]["facts"][:1]),
        quality=QUALITY_PATH,
        badges="".join(
            '<li class="badge"><span class="badge-seal">{}</span><div><strong>{}</strong><span>{}</span></div></li>'.format(
                icon("lock" if "Cyber" in c["code"] else "award"), esc(c["code"]), esc(c["title"]))
            for c in C.QUALITY["certs"]),
        cta=cta(r),
    )
    write_page("", "Fluid Systems Engineering | Element Digital Engineering", SITE["description"], main,
               jsonld=[org_jsonld()])


def build_what_we_do():
    r = "../"
    main = """{hero}
<section class="section">
  <div class="container split">
    <div data-reveal>
      <p class="eyebrow">Systems we engineer</p>
      <h2>Aerospace origins, cross-industry reach.</h2>
    </div>
    <div class="prose lead" data-reveal>
      <p>Originating in the aerospace industry, we have a recognised capability in designing fuel, air, hydraulic, inerting and engine systems. Many of our engineers hold 10 to 25 years' experience.</p>
      <p>We have since expanded to offer that expertise to a range of industries reliant on fluid systems, including <a href="../hydrogen/">hydrogen</a>, renewable energy, oil and gas, water and nuclear.</p>
      {systems}
    </div>
  </div>
</section>
<section class="section section--tint">
  <div class="container">
    <div class="section-head" data-reveal><div><p class="eyebrow">Capabilities</p><h2>How we can help</h2></div></div>
    <ul class="card-grid" data-reveal-group>{cards}</ul>
  </div>
</section>
<section class="section">
  <div class="container">
    <div class="section-head" data-reveal>
      <div><p class="eyebrow">The lifecycle</p><h2>Concept to certification.</h2>
      <p>Assess an existing design, or take a fluid system or equipment package through the complete product development cycle.</p></div>
    </div>
    {lifecycle}
  </div>
</section>
{cta}""".format(
        hero=page_hero(r, "A single source for fluid systems engineering.",
                       "We deliver solutions for the successful development of fluid systems and equipment from "
                       "concept to manufacture: mechanical design, simulation, analysis, prototyping, technical "
                       "writing and testing.",
                       eyebrow="What we do", crumbs=[("Home", ""), ("What we do", "what-we-do")]),
        systems=chips(C.SYSTEMS),
        cards="".join(service_card(r, s) for s in C.SERVICES),
        lifecycle=lifecycle(),
        cta=cta(r),
    )
    write_page("what-we-do", "What We Do | Element Digital Engineering",
               "Fluid system mechanical design, simulation, analysis, prototyping, technical writing and testing, "
               "from concept to manufacture and certification.",
               main, active="what-we-do",
               jsonld=[crumbs_jsonld([("Home", ""), ("What we do", "what-we-do")])])


def build_service(i, s):
    r = "../../"
    path = "what-we-do/" + s["slug"]
    crumbs = [("Home", ""), ("What we do", "what-we-do"), (s["title"], path)]

    lists = "".join("<h2>{}</h2>{}".format(esc(h), checklist(items)) for h, items in s["lists"])
    if s.get("standards"):
        lists += "<h2>Standards we work to</h2>" + chips(s["standards"])

    aside_nav = "".join(
        '<li><a href="{r}what-we-do/{slug}/"{cur}>{ic}{t}</a></li>'.format(
            r=r, slug=o["slug"], cur=' aria-current="page"' if o is s else "", ic=icon(o["icon"]), t=esc(o["title"]))
        for o in C.SERVICES)

    prev_s = C.SERVICES[i - 1] if i > 0 else None
    next_s = C.SERVICES[i + 1] if i + 1 < len(C.SERVICES) else None
    pager = ""
    if prev_s:
        pager += '<a class="prev" href="{}what-we-do/{}/"><small>Previous</small><strong>{}</strong></a>'.format(r, prev_s["slug"], esc(prev_s["title"]))
    if next_s:
        pager += '<a class="next" href="{}what-we-do/{}/"><small>Next</small><strong>{}</strong></a>'.format(r, next_s["slug"], esc(next_s["title"]))

    main = """{hero}
<section class="section">
  <div class="container service-layout">
    <article class="service-body">
      <div class="prose">{body}</div>
      {lists}
    </article>
    <aside class="aside" aria-label="Enquiries and other capabilities">
      <div class="aside-card aside-card--dark on-dark">
        <h2>Discuss a project</h2>
        <p>Talk to an engineer about your system or programme.</p>
        <ul class="aside-contact">
          <li><a href="mailto:{email}">{mail}{email}</a></li>
          <li><a href="tel:{tel}">{phone}{phone_d}</a></li>
        </ul>
        <a class="btn btn-primary" href="{r}contact/">Send an enquiry</a>
      </div>
      <div class="aside-card">
        <h2>Capabilities</h2>
        <ul class="aside-nav">{aside_nav}</ul>
      </div>
    </aside>
  </div>
</section>
<section class="section section--tint section--tight">
  <div class="container">
    <div class="section-head"><div><p class="eyebrow">Related</p><h2>Related capabilities</h2></div></div>
    <ul class="card-grid card-grid--3" data-reveal-group>{related}</ul>
    <nav class="pager mt-l" aria-label="More capabilities">{pager}</nav>
  </div>
</section>
{cta}""".format(
        hero=page_hero(r, esc(s["title"]), esc(s["lead"]), crumbs=crumbs, icon_name=s["icon"]),
        body="".join("<p>{}</p>".format(p) for p in s["body"]),
        lists=lists,
        email=SITE["email"], tel=SITE["phone_href"], phone_d=esc(SITE["phone_display"]),
        mail=icon("mail"), phone=icon("phone"), r=r, aside_nav=aside_nav,
        related="".join(service_card(r, BY_SLUG[x]) for x in s["related"]),
        pager=pager,
        cta=cta(r),
    )
    write_page(path, "{} | Element Digital Engineering".format(s["title"]), s["summary"], main,
               active="what-we-do/" + s["slug"], jsonld=[crumbs_jsonld(crumbs)])


def build_hydrogen():
    r = "../"
    h = C.HYDROGEN
    p = h["project"]
    main = """{hero}
<section class="section">
  <div class="container split">
    <div data-reveal>
      <p class="eyebrow">Our expertise</p>
      <h2>A demanding fluid, squarely in our core skills.</h2>
    </div>
    <div data-reveal>
      <div class="prose lead">{body}</div>
      <h3 class="mt-m" style="font-size:1.125rem;margin-bottom:18px">Capabilities</h3>
      {caps}
    </div>
  </div>
</section>
<section class="section section--tight" style="padding-top:0">
  <div class="container">
    <div class="feature on-dark" data-reveal>
      <div class="feature-body">
        <p class="eyebrow">{p_label}</p>
        <h2>{p_title}</h2>
        <p>{p_body}</p>
      </div>
      <div class="feature-art">
        <img src="{r}assets/img/flow.svg" alt="" width="1600" height="900" loading="lazy">
        <div class="feature-facts">{facts}</div>
      </div>
    </div>
  </div>
</section>
<section class="section section--tint section--tight">
  <div class="container">
    <div class="section-head"><div><p class="eyebrow">Related</p><h2>Capabilities behind our hydrogen work</h2></div></div>
    <ul class="card-grid card-grid--3" data-reveal-group>{related}</ul>
  </div>
</section>
{cta}""".format(
        hero=page_hero(r, "Liquid hydrogen fuel systems for zero-emission flight.", h["lead"],
                       crumbs=[("Home", ""), ("Hydrogen", "hydrogen")], icon_name="h2"),
        body="".join("<p>{}</p>".format(x) for x in h["body"]),
        caps=checklist(h["capabilities"]),
        p_label=esc(p["label"]), p_title=esc(p["title"]), p_body=p["body"], r=r,
        facts="".join('<div class="fact"><strong>{}</strong><span>{}</span></div>'.format(a, esc(b)) for a, b in p["facts"]),
        related="".join(service_card(r, BY_SLUG[x]) for x in ["fluid-modelling", "testing", "equipment-design"]),
        cta=cta(r, "Working on hydrogen?", "Talk to our team about liquid or gaseous hydrogen systems, modelling and testing."),
    )
    write_page("hydrogen", "Hydrogen | Element Digital Engineering",
               "Gaseous and liquid hydrogen fuel system engineering, with a dedicated facility and in-house "
               "liquefaction plant at Kemble, UK.",
               main, active="hydrogen", jsonld=[crumbs_jsonld([("Home", ""), ("Hydrogen", "hydrogen")])])


def build_about():
    r = "../"
    timeline = "".join(
        '<li><time>{}</time><h3>{}</h3><p>{}</p></li>'.format(esc(y), esc(t), esc(d)) for y, t, d in C.TIMELINE)
    diffs = "".join("<li><h3>{}</h3><p>{}</p></li>".format(esc(t), esc(d)) for t, d in C.DIFFERENTIATORS)
    main = """{hero}
<section class="section">
  <div class="container split">
    <div data-reveal>
      <p class="eyebrow">Our story</p>
      <h2>Built on a core of fuel system and fluid mechanical experts.</h2>
    </div>
    <div class="prose lead" data-reveal>
      <p>Filton Systems Engineering was founded in Bristol in 2012, a few miles from Filton, home to more than a century of British aerospace, and the place that gave the company its name.</p>
      <p>Originating in the aerospace industry, we built a recognised capability in designing fuel, air, hydraulic, inerting and engine systems, with many of our engineers holding 10 to 25 years' experience. We have since expanded to offer that expertise to other industries reliant on fluid systems, including renewable energy, oil and gas, water and nuclear.</p>
      <p>In 2023 we joined Element Materials Technology and became part of Element Digital Engineering. Headquartered in Bristol, with a dedicated workforce in both the UK and US, we combine the focus of a specialist consultancy with the reach of a global group.</p>
    </div>
  </div>
</section>
<section class="section section--tint">
  <div class="container">
    <div class="section-head" data-reveal><div><p class="eyebrow">Milestones</p><h2>From Bristol consultancy to global group.</h2></div></div>
    <ol class="timeline" data-reveal-group>{timeline}</ol>
  </div>
</section>
<section class="section section--dark on-dark">
  <div class="container">
    <div class="section-head" data-reveal><div><p class="eyebrow">Why work with us</p><h2>Specialists you can rely on.</h2></div></div>
    <ul class="diff-grid" data-reveal-group>{diffs}</ul>
  </div>
</section>
<section class="section">
  <div class="container split">
    <div data-reveal>
      <p class="eyebrow">Part of Element</p>
      <h2>Specialist focus, global reach.</h2>
    </div>
    <div class="prose lead" data-reveal>
      <p>Element Materials Technology is a global testing, inspection and certification business. Element Digital Engineering was formed in 2022 to support customers across a diverse range of markets through the complete engineering lifecycle.</p>
      <p>Filton Systems Engineering's expertise in systems engineering, safety, reliability and design, along with simulation and modelling, complements Element's offering across the complete product development cycle.</p>
      <p><a class="link-arrow" href="https://www.element.com" rel="noopener">Visit element.com {ext}</a></p>
    </div>
  </div>
</section>
{cta}""".format(
        hero=page_hero(r, "Specialist fluid systems engineers, with global reach.",
                       "An engineering consultancy founded in Bristol in 2012, now part of Element Materials Technology.",
                       crumbs=[("Home", ""), ("About", "about")], eyebrow="About us", extra=stats_block()),
        timeline=timeline, diffs=diffs, ext=icon("external"), cta=cta(r),
    )
    write_page("about", "About Us | Element Digital Engineering",
               "Founded in Bristol in 2012 as Filton Systems Engineering, now part of Element Materials Technology. "
               "Fluid systems specialists with teams in the UK and US.",
               main, active="about", jsonld=[crumbs_jsonld([("Home", ""), ("About", "about")])])


def build_quality():
    r = "../"
    q = C.QUALITY
    certs = "".join(
        '<li class="cert"><span class="badge-seal">{}</span><code>{}</code><h3>{}</h3><p>{}</p></li>'.format(
            icon("lock" if "Cyber" in c["code"] else "award"), esc(c["code"]), esc(c["title"]), esc(c["text"]))
        for c in q["certs"])
    docs = "".join(
        '<li><a href="{r}{href}"><span>{label}<small>PDF</small></span>{dl}</a></li>'.format(
            r=r, href=href, label=esc(label), dl=icon("download"))
        for label, href in q["documents"])
    main = """{hero}
<section class="section">
  <div class="container">
    <div class="section-head" data-reveal><div><p class="eyebrow">Certification</p><h2>Independently assessed.</h2></div></div>
    <ul class="cert-grid" data-reveal-group>{certs}</ul>
  </div>
</section>
<section class="section section--dark on-dark">
  <div class="container split--even split">
    <div data-reveal>
      <p class="eyebrow">Scope</p>
      <h2>What our certification covers.</h2>
      <p class="lead mt-s">{scope}</p>
    </div>
    <div data-reveal>
      <h3 style="font-size:1.125rem">Certificates</h3>
      <ul class="doc-list">{docs}</ul>
    </div>
  </div>
</section>
{cta}""".format(
        hero=page_hero(r, "Quality and certification.", esc(q["lead"]),
                       crumbs=[("Home", ""), ("Quality", QUALITY_PATH)], icon_name="shield"),
        certs=certs, scope=esc(q["scope"]), docs=docs, cta=cta(r),
    )
    write_page(QUALITY_PATH, "Quality and Certification | Element Digital Engineering",
               "EN 9100:2018 and ISO 9001:2015 certified quality management, and Cyber Essentials Plus certified.",
               main, active="quality", jsonld=[crumbs_jsonld([("Home", ""), ("Quality", QUALITY_PATH)])])


def build_contact():
    r = "../"
    options = "".join('<option>{}</option>'.format(esc(t)) for t in
                      ["General enquiry"] + [s["title"] for s in C.SERVICES] + ["Hydrogen", "Careers"])
    addr = "<br>".join(esc(a) for a in SITE["address_lines"])
    main = """{hero}
<section class="section">
  <div class="container contact-grid">
    <ul class="contact-list" data-reveal-group>
      <li class="contact-item"><span class="card-icon">{mail}</span><div><h2>Email</h2><a href="mailto:{email}">{email}</a></div></li>
      <li class="contact-item"><span class="card-icon">{phone}</span><div><h2>Telephone</h2><a href="tel:{tel}">{phone_d}</a></div></li>
      <li class="contact-item"><span class="card-icon">{pin}</span><div><h2>Head office</h2><address>{addr}</address>
        <a class="link-arrow" href="{maps}" rel="noopener">Get directions {arrow}</a></div></li>
    </ul>
    <form class="form" data-enquiry data-to="{email}" action="mailto:{email}" method="post" enctype="text/plain" data-reveal>
      <h2>Send an enquiry</h2>
      <p>Tell us a little about your system or programme and we will be in touch.</p>
      <div class="form-grid">
        <div class="field"><label for="f-name">Name</label><input id="f-name" name="name" autocomplete="name" required></div>
        <div class="field"><label for="f-company">Company <span>(optional)</span></label><input id="f-company" name="company" autocomplete="organization"></div>
        <div class="field"><label for="f-email">Email</label><input id="f-email" name="email" type="email" autocomplete="email" required></div>
        <div class="field"><label for="f-phone">Phone <span>(optional)</span></label><input id="f-phone" name="phone" type="tel" autocomplete="tel"></div>
        <div class="field full"><label for="f-topic">Area of interest</label><select id="f-topic" name="topic">{options}</select></div>
        <div class="field full"><label for="f-message">Message</label><textarea id="f-message" name="message" required></textarea></div>
      </div>
      <div class="form-foot">
        <button class="btn btn-dark" type="submit">Send enquiry {arrow}</button>
        <p>This opens your email app with your message ready to send. Prefer to write directly? Email <a href="mailto:{email}">{email}</a>.</p>
      </div>
      <p class="form-status" role="status" aria-live="polite"></p>
    </form>
  </div>
</section>""".format(
        hero=page_hero(r, "Let's talk about your fluid system.",
                       "Whether you have a specific technical challenge or a whole programme to support, our "
                       "engineers would be glad to hear from you.",
                       crumbs=[("Home", ""), ("Contact", "contact")], eyebrow="Contact"),
        mail=icon("mail"), phone=icon("phone"), pin=icon("pin"), arrow=icon("arrow"),
        email=SITE["email"], tel=SITE["phone_href"], phone_d=esc(SITE["phone_display"]),
        addr=addr, maps=SITE["maps_url"], options=options,
    )
    write_page("contact", "Contact Us | Element Digital Engineering",
               "Contact Element Digital Engineering (formerly Filton Systems Engineering), Unit 13 Apex Court, "
               "Woodlands, Bristol BS32 4JT. Email global@filtonsys.com.",
               main, active="contact",
               jsonld=[org_jsonld(), crumbs_jsonld([("Home", ""), ("Contact", "contact")])])


def build_404():
    r = "/"
    main = page_hero(
        r, "Page not found.",
        "The page you were looking for has moved or no longer exists. Our website has been redesigned, "
        "so some older links may have changed.",
        eyebrow="Error 404",
        actions='<div class="actions"><a class="btn btn-primary" href="/">Home {a}</a>'
                '<a class="btn btn-ghost" href="/what-we-do/">What we do</a>'
                '<a class="btn btn-ghost" href="/contact/">Contact us</a></div>'.format(a=icon("arrow")),
    )
    write_page("404.html", "Page Not Found | Element Digital Engineering",
               "The page you were looking for could not be found.", main, root=r, index=False)


# ---------------------------------------------------------------------------
# Static extras
# ---------------------------------------------------------------------------
def build_assets():
    img = OUT / "assets" / "img"
    img.mkdir(parents=True, exist_ok=True)
    (img / "flow.svg").write_text(flowfield.build_svg(), encoding="utf-8")
    (img / "favicon.svg").write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32">{}</svg>'.format(MARK_INNER), encoding="utf-8")


def build_sitemap():
    urls = "".join(
        "  <url><loc>{}/{}</loc><lastmod>{}</lastmod></url>\n".format(SITE["url"], p + "/" if p else "", TODAY)
        for p in PAGES)
    (OUT / "sitemap.xml").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n' + urls + "</urlset>\n", encoding="utf-8")
    (OUT / "robots.txt").write_text("User-agent: *\nAllow: /\n\nSitemap: {}/sitemap.xml\n".format(SITE["url"]),
                                    encoding="utf-8")


def main():
    build_assets()
    build_home()
    build_what_we_do()
    for i, s in enumerate(C.SERVICES):
        build_service(i, s)
    build_hydrogen()
    build_about()
    build_quality()
    build_contact()
    build_404()
    build_sitemap()
    print("Built {} pages into {}".format(len(PAGES) + 1, OUT.relative_to(HERE)))
    missing = [href for _, href in C.QUALITY["documents"] if not (OUT / href).exists()]
    for href in missing:
        print("WARNING: public/{} is missing; copy it from the old site (see README).".format(href))


if __name__ == "__main__":
    main()
