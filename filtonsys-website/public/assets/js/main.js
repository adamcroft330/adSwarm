(function () {
  "use strict";

  var header = document.querySelector("[data-header]");
  var toggle = document.querySelector(".nav-toggle");
  var mobile = window.matchMedia("(max-width: 1023px)");

  // Solid header once the page scrolls past the top of the hero.
  function onScroll() {
    if (header) header.classList.toggle("is-scrolled", window.scrollY > 8);
  }
  onScroll();
  window.addEventListener("scroll", onScroll, { passive: true });

  // Mobile menu.
  function setMenu(open) {
    if (!header || !toggle) return;
    header.classList.toggle("is-open", open);
    document.body.classList.toggle("nav-open", open);
    toggle.setAttribute("aria-expanded", String(open));
    toggle.querySelector(".sr-only").textContent = open ? "Close menu" : "Open menu";
  }
  if (toggle) {
    toggle.addEventListener("click", function () {
      setMenu(toggle.getAttribute("aria-expanded") !== "true");
    });
  }
  document.addEventListener("keydown", function (e) {
    if (e.key !== "Escape") return;
    if (header && header.classList.contains("is-open")) {
      setMenu(false);
      toggle.focus();
    }
    document.querySelectorAll(".has-sub.is-open").forEach(function (li) { closeSub(li); });
  });
  mobile.addEventListener("change", function (e) { if (!e.matches) setMenu(false); });

  // Capabilities sub-menu: hover/focus on desktop, button on touch and mobile.
  function closeSub(li) {
    li.classList.remove("is-open");
    var b = li.querySelector(".sub-toggle");
    if (b) b.setAttribute("aria-expanded", "false");
  }
  document.querySelectorAll(".has-sub").forEach(function (li) {
    var btn = li.querySelector(".sub-toggle");
    if (!btn) return;
    btn.addEventListener("click", function () {
      var open = !li.classList.contains("is-open");
      li.classList.toggle("is-open", open);
      btn.setAttribute("aria-expanded", String(open));
    });
    li.addEventListener("focusout", function (e) {
      if (!mobile.matches && !li.contains(e.relatedTarget)) closeSub(li);
    });
  });
  document.addEventListener("click", function (e) {
    if (mobile.matches) return;
    document.querySelectorAll(".has-sub.is-open").forEach(function (li) {
      if (!li.contains(e.target)) closeSub(li);
    });
  });

  // Reveal on scroll.
  var targets = document.querySelectorAll("[data-reveal], [data-reveal-group]");
  if ("IntersectionObserver" in window) {
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (entry) {
        if (entry.isIntersecting) {
          entry.target.classList.add("is-visible");
          io.unobserve(entry.target);
        }
      });
    }, { rootMargin: "0px 0px -8% 0px", threshold: 0.08 });
    targets.forEach(function (el) { io.observe(el); });
  } else {
    targets.forEach(function (el) { el.classList.add("is-visible"); });
  }

  // Contact form: compose an email in the visitor's mail client.
  // Swap this for a form service (see README) to receive submissions directly.
  var form = document.querySelector("[data-enquiry]");
  if (form) {
    form.addEventListener("submit", function (e) {
      e.preventDefault();
      if (!form.reportValidity()) return;
      var d = new FormData(form);
      var topic = d.get("topic") || "General enquiry";
      var lines = [
        d.get("message"),
        "",
        "--",
        d.get("name"),
        d.get("company"),
        d.get("email"),
        d.get("phone")
      ].filter(function (v, i) { return i < 3 || v; });
      var href = "mailto:" + form.dataset.to +
        "?subject=" + encodeURIComponent("Enquiry: " + topic) +
        "&body=" + encodeURIComponent(lines.join("\n"));
      window.location.href = href;
      var status = form.querySelector(".form-status");
      if (status) {
        status.textContent = "Your email app should now open with your message ready to send. " +
          "If nothing happens, email us directly at " + form.dataset.to + ".";
      }
    });
  }

  var year = document.querySelector("[data-year]");
  if (year) year.textContent = new Date().getFullYear();
})();
