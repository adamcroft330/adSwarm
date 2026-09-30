# filtonsys.com redesign

A replacement website for **Element Digital Engineering** (formerly Filton Systems
Engineering). It's a fast static site with no framework, no tracking and no
third-party requests. `public/` is the finished site and can go on any static host.

## Quick start

```bash
python3 build.py                        # regenerate public/ from content.py
python3 -m http.server 8000 -d public   # preview at http://localhost:8000
```

`build.py` only needs Python 3's standard library.

## Layout

| Path | What it is |
| --- | --- |
| `content.py` | All site copy: services, industries, timeline, quality, contact details. **Edit text here.** |
| `build.py` | Page templates. Writes every page to `public/`. |
| `flowfield.py` | Draws the hero artwork. It computes real potential-flow streamlines around a Joukowski airfoil (Kutta condition applied) and writes `public/assets/img/flow.svg`. |
| `public/assets/css/styles.css` | All styles, with design tokens at the top. |
| `public/assets/js/main.js` | Mobile menu, capabilities dropdown, scroll reveal and the contact form (about 4 KB, no dependencies). |
| `public/assets/fonts/` | Self-hosted Inter and JetBrains Mono (SIL OFL; licences included). |
| `tools/render-images.js` | Optional. Re-renders `og.png`, `apple-touch-icon.png` and `logo.png` with Playwright. |

## Pages and URLs

The old site's URLs are kept, so existing links, bookmarks and search rankings
still work:

| URL | Page |
| --- | --- |
| `/` | Home |
| `/what-we-do/` | Capabilities overview |
| `/what-we-do/fluid-modelling/` | Fluid modelling |
| `/what-we-do/surge-analysis/` | Surge analysis |
| `/what-we-do/electro-statics/` | Electrostatics |
| `/what-we-do/equipment-design/` | Equipment design |
| `/what-we-do/certification/` | Aircraft certification |
| `/what-we-do/testing/` | Test programmes and test rigs |
| `/what-we-do/skills/` | Core skills |
| `/about/` | About |
| `/en9100-quality-certificate-and-cyber-essentials-plus/` | Quality |
| `/contact/` | Contact |
| `/what-we-do/ice-and-water-in-fuel/` | **New.** Ice and water in fuel |
| `/hydrogen/` | **New.** Hydrogen capability (Kemble facility, Safe Flight) |
| `/404.html` | Not found page |

Each page is written as `<path>/index.html`. Hosts that serve directory indexes
(Netlify, Cloudflare Pages, GitHub Pages, S3/CloudFront, Apache and nginx) will
serve both `/about` and `/about/`. Internal links are relative, so the site also
works from a sub-folder, for example a GitHub Pages project site used for review.

## Before go-live

The current filtonsys.com couldn't be fetched directly while this was built.
The copy comes from the site's search-indexed text and public press releases
about the Element acquisition and the GKN Safe Flight project. Please check:

- [ ] **Certificate PDFs.** Copy `QMS-Certificate_2023.pdf` and
      `FS-738719-052024.pdf` from the current site's `/assets/` folder into
      `public/assets/`. The build prints a warning until they're there. Check the
      link labels in `content.py` (`QUALITY["documents"]`) and update them if the
      certificates have been renewed.
- [ ] **Copy review.** An engineer should read every page, especially
      `ice-and-water-in-fuel` (a new page, written from the specialism named on
      the old site), `hydrogen` (is Kemble still operating?), and the deliverable
      lists on each capability page.
- [ ] **Legal line.** The footer says "Filton Systems Engineering Ltd, part of
      Element. Registered in England and Wales, company no. 08230492." Confirm
      the trading entity and wording with Element's legal and brand team.
- [ ] **Brand.** The mark in the header is a placeholder (three streamlines).
      Swap in the official Element Digital Engineering logo, and colours if
      Element's brand guidelines require them. Colours are CSS variables at the
      top of `styles.css`, and the mark is `MARK_INNER` in `build.py`.
- [ ] **Contact form.** By default it opens the visitor's email app with the
      message filled in, so no server is needed. To receive submissions
      directly, point the `<form>` in `build_contact()` at a form service
      (Netlify Forms, Formspree, Basin and so on) and remove the `data-enquiry`
      attribute so `main.js` leaves it alone.
- [ ] **Other old URLs.** If the old site had pages not listed above (news,
      careers and so on), add redirects on the host, or they'll get the 404 page.
- [ ] **Analytics.** None is included. If you add any, add a cookie notice too.
- [ ] **Search Console.** After launch, submit `https://filtonsys.com/sitemap.xml`.

## Deploying

Upload the contents of `public/` to the web root. Set the host's 404 page to
`/404.html`. Examples:

- **Netlify / Cloudflare Pages:** build command `python3 build.py`, publish directory
  `filtonsys-website/public`.
- **Apache:** add `ErrorDocument 404 /404.html` to `.htaccess`.
- **nginx:** `error_page 404 /404.html;`

## Built in

- Responsive from 320 px up. Mobile menu, and a keyboard-accessible capabilities
  dropdown (Escape closes it).
- WCAG AA text contrast, a skip link, visible focus styles, semantic landmarks,
  and support for `prefers-reduced-motion` (the flow animation and scroll
  reveals stop).
- A canonical URL, meta description, Open Graph and Twitter card on every
  page, plus `Organization` and `BreadcrumbList` JSON-LD, `sitemap.xml` and
  `robots.txt`.
- Hashed CSS and JS query strings, so browsers pick up new versions after
  each build. Fonts are preloaded with `font-display: swap`.
- About 750 KB for the whole site, including fonts and the share image.
