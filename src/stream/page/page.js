// Visor's live view page: the cameras screen and a tab per camera. Everything shown comes from
// the status document (GET /status), read twice a second; an action's answer is shown for a few
// seconds and then cleared. No library and nothing loaded from elsewhere: a robot's network has
// no internet.
"use strict";

const READ_EVERY_MS = 500;
const ANSWER_FOR_MS = 4000;

let documentNow = null;
let shown = null; // "cameras", or a camera's name
const cards = new Map(); // camera name -> its card's elements

const $ = (selector, within) => (within || document).querySelector(selector);

// Every verdict's label, as the page shows it: one table for all of them.
const LABELS = {
  good: "Good",
  too_dark: "Too dark",
  too_bright: "Too bright",
  no_tag: "No tag in view",
  sharp: "Sharp",
  soft: "Soft",
  needs_focusing: "Needs focusing",
  cant_judge: "Can't judge",
  calibrated: "Calibrated",
  missing: "Missing",
};

function labelOf(verdict) {
  return LABELS[verdict] || verdict;
}

// A verdict's mark, for its colour: attention, unknown or fine.
function markOf(verdict) {
  switch (verdict) {
    case "good":
    case "sharp":
    case "calibrated":
      return "fine";
    case "no_tag":
    case "cant_judge":
      return "unknown";
    default:
      return "attention";
  }
}

function setVerdict(element, verdict) {
  element.textContent = labelOf(verdict);
  element.className = "verdict " + markOf(verdict);
}

// Shows an action's answer, then clears it.
function answer(element, text) {
  element.textContent = text;
  clearTimeout(element.clearing);
  element.clearing = setTimeout(() => { element.textContent = ""; }, ANSWER_FOR_MS);
}

function post(path, into) {
  fetch(path, { method: "POST" })
    .then((reply) => reply.text())
    .then((text) => answer(into, text.trim()))
    .catch((error) => answer(into, String(error)));
}

// Which screen the address asks for: the cameras screen, or #camera/<name>.
function wanted() {
  const hash = decodeURIComponent(location.hash.slice(1));
  return hash.startsWith("camera/") ? hash.slice("camera/".length) : "cameras";
}

function cameraNamed(name) {
  return documentNow && documentNow.cameras.find((camera) => camera.name === name);
}

// ---- The cameras screen ----

function card(camera) {
  let made = cards.get(camera.name);
  if (made) return made;
  const link = document.createElement("a");
  link.className = "card";
  link.href = "#camera/" + encodeURIComponent(camera.name);
  const picture = document.createElement("img");
  picture.alt = camera.name + "'s live view";
  const name = document.createElement("p");
  name.className = "name";
  const list = document.createElement("ul");
  list.className = "checks";
  link.append(picture, name, list);
  $("#grid").append(link);
  made = { link, picture, name, list };
  cards.set(camera.name, made);
  return made;
}

// Sizes the cameras screen's cards: each picture as wide as it can be, in its own shape (so no
// black bars), with the grid's columns chosen so every card, its list included, fits the window.
// On a phone's width the cards are one column and the screen scrolls.
function layOut() {
  const grid = $("#grid");
  const made = [...cards.values()];
  if (made.length === 0 || shown !== "cameras") return;
  if (matchMedia("(max-width: 760px)").matches) {
    for (const card of made) card.link.style.width = "";
    return;
  }
  const gap = 8;
  const area = { width: grid.clientWidth, height: $("#cameras").clientHeight - 16 };
  const aspect = Math.max(...documentNow.cameras.map((c) => c.height / c.width));
  // A card's list wraps differently at each width, so what it takes besides its picture is
  // measured again after each sizing, a few times over, until the sizes hold.
  for (let pass = 0; pass < 3; pass++) {
    const besides = {
      width: Math.max(...made.map((card) => card.link.offsetWidth - card.picture.offsetWidth)),
      height: Math.max(...made.map((card) => card.link.offsetHeight - card.picture.offsetHeight)),
    };
    let best = 0;
    for (let columns = 1; columns <= made.length; columns++) {
      const rows = Math.ceil(made.length / columns);
      const byWidth = (area.width - gap * (columns - 1)) / columns - besides.width;
      const byHeight = ((area.height - gap * (rows - 1)) / rows - besides.height) / aspect;
      best = Math.max(best, Math.floor(Math.min(byWidth, byHeight)));
    }
    for (const card of made) card.link.style.width = Math.max(best, 120) + "px";
  }
}

function showCameras() {
  const cameras = documentNow.cameras;
  $("#none").hidden = cameras.length > 0;
  for (const camera of cameras) {
    const made = card(camera);
    // A stream holds a connection open: only the screen shown streams.
    if (shown === "cameras" && made.picture.getAttribute("src") !== camera.stream) {
      made.picture.src = camera.stream;
    }
    made.name.textContent = camera.name;
    if (camera.serial) {
      const serial = document.createElement("span");
      serial.className = "serial";
      serial.textContent = " (serial " + camera.serial + ")";
      made.name.append(serial);
    }
    made.picture.style.aspectRatio = camera.width + " / " + camera.height;
    made.list.replaceChildren(
      ...camera.checks.map((check) => {
        const item = document.createElement("li");
        item.className = check.mark;
        item.textContent = check.say;
        return item;
      }),
    );
  }
  layOut();
}

function stopCameraStreams() {
  for (const made of cards.values()) made.picture.removeAttribute("src");
}

// ---- A camera's tab ----

function hand() {
  return $("#hand");
}

function exposureOfSlider(form) {
  const least = Number(form.dataset.least);
  const most = Number(form.dataset.most);
  const us = least * Math.pow(most / least, form.exposureSlider.value / 1000);
  return Math.max(least, Math.min(most, Math.round(us / 100) * 100));
}

function sliderOfExposure(form, us) {
  const least = Number(form.dataset.least);
  const most = Number(form.dataset.most);
  if (!(most > least) || !(us > 0)) return 0;
  return Math.round((1000 * Math.log(Math.max(us, least) / least)) / Math.log(most / least));
}

function showCamera(camera) {
  const tab = $("#camera");
  const stream = $("#tab-stream");
  if (stream.getAttribute("src") !== camera.stream) {
    stream.src = camera.stream;
    stream.alt = camera.name + "'s live view";
  }

  const brightness = camera.brightness;
  const row = $("#brightness", tab);
  setVerdict($(".verdict", row), brightness.verdict);
  $(".say", row).textContent = brightness.say;
  const exposure = brightness.auto_exposure
    ? "automatic exposure"
    : brightness.exposure_us == null
      ? "exposure not known"
      : "exposure " + (brightness.exposure_us / 1000).toFixed(1) + " ms";
  const gain = brightness.gain == null ? "no gain control" : "gain " + brightness.gain;
  const white = brightness.white == null ? "" : ", the tag's white " + brightness.white;
  $(".facts", row).textContent = "In effect: " + exposure + ", " + gain + white + ".";
  $("#set").hidden = !brightness.can_set;
  $("#by-hand").hidden = !brightness.can_set;
  $(".setting", row).textContent = brightness.setting_say || "";
  $(".changed", row).textContent = brightness.settings_say || "";
  showRanges(brightness);

  const focus = camera.focus;
  const focusRow = $("#focus", tab);
  setVerdict($(".verdict", focusRow), focus.verdict);
  $(".say", focusRow).textContent = focus.say;
  $(".bar div", focusRow).style.width = ((focus.bar || 0) * 100).toFixed(0) + "%";
  $(".bar", focusRow).setAttribute("aria-valuenow", focus.bar == null ? 0 : focus.bar);
  $(".small", focusRow).textContent = focus.small_print;

  const lens = $("#lens", tab);
  setVerdict($(".verdict", lens), camera.lens.calibrated ? "calibrated" : "missing");
  $(".say", lens).textContent = camera.lens.say;

  const facts = [
    ["Name", camera.name],
    ["Index", String(camera.index)],
    ["Serial", camera.serial || "none reported"],
    ["Device", camera.device],
    ["Link", camera.link || "not known yet"],
    ["Size", camera.width + " x " + camera.height],
    [
      "Rate",
      camera.fps_asked + " frames a second asked for" +
        (camera.fps == null ? "" : ", " + camera.fps.toFixed(1) + " measured"),
    ],
  ];
  $("#facts").replaceChildren(
    ...facts.flatMap(([term, value]) => {
      const dt = document.createElement("dt");
      dt.textContent = term;
      const dd = document.createElement("dd");
      dd.textContent = value;
      return [dt, dd];
    }),
  );
}

// The by-hand controls keep to the camera's own ranges, and follow what's in effect while
// nobody is changing them.
function showRanges(brightness) {
  const form = hand();
  const editing = form.contains(document.activeElement);
  const exposureRange = brightness.exposure_range_us;
  if (exposureRange) {
    form.dataset.least = Math.max(exposureRange[0], 1);
    form.dataset.most = exposureRange[1];
    $(".exposure-range", form).textContent =
      "the camera takes " + (exposureRange[0] / 1000).toFixed(1) + " to " +
      (exposureRange[1] / 1000).toFixed(1) + " ms";
  } else {
    $(".exposure-range", form).textContent = "the camera's range isn't known yet";
  }
  const gainRange = brightness.gain_range;
  if (gainRange) {
    form.gainSlider.min = gainRange[0];
    form.gainSlider.max = gainRange[1];
    $(".gain-range", form).textContent = "the camera takes " + gainRange[0] + " to " + gainRange[1];
  } else {
    $(".gain-range", form).textContent = "no gain control";
  }
  if (!editing && brightness.exposure_us != null && !brightness.auto_exposure) {
    form.exposure.value = (brightness.exposure_us / 1000).toFixed(1);
    form.exposureSlider.value = sliderOfExposure(form, brightness.exposure_us);
  }
  if (!editing && brightness.gain != null) {
    form.gain.value = brightness.gain;
    form.gainSlider.value = brightness.gain;
  }
}

function applyByHand(event) {
  if (event) event.preventDefault();
  const camera = cameraNamed(shown);
  if (!camera) return;
  const form = hand();
  const asked = [];
  if (form.exposure.value) asked.push("exposure_us=" + Math.round(Number(form.exposure.value) * 1000));
  if (form.gain.value) asked.push("gain=" + encodeURIComponent(form.gain.value));
  post(camera.by_hand + "?" + asked.join("&"), $("#brightness .answer"));
}

// ---- Screens ----

function showTabs() {
  const links = [["Cameras", "#", "cameras"]].concat(
    documentNow.cameras.map((camera) => [
      camera.name,
      "#camera/" + encodeURIComponent(camera.name),
      camera.name,
    ]),
  );
  $("#tabs").replaceChildren(
    ...links.map(([label, href, which]) => {
      const link = document.createElement("a");
      link.href = href;
      link.textContent = label;
      if (which === shown) link.setAttribute("aria-current", "page");
      return link;
    }),
  );
}

let cornerTimer = null;

function refreshCorner() {
  const camera = cameraNamed(shown);
  if (!camera) return;
  const corner = $("#focus .corner");
  const next = new Image();
  next.onload = () => { corner.src = next.src; };
  next.src = camera.corner + "?at=" + Date.now();
}

function showScreen() {
  if (!documentNow) return;
  const want = wanted();
  const camera = want === "cameras" ? null : cameraNamed(want);
  const now = camera ? camera.name : "cameras";
  if (now !== shown) {
    shown = now;
    $("#cameras").hidden = shown !== "cameras";
    $("#camera").hidden = shown === "cameras";
    if (shown === "cameras") {
      $("#tab-stream").removeAttribute("src");
      $("#focus .corner").removeAttribute("src");
      clearInterval(cornerTimer);
      cornerTimer = null;
    } else {
      stopCameraStreams();
      if (!cornerTimer) cornerTimer = setInterval(refreshCorner, READ_EVERY_MS);
    }
  }
  showTabs();
  if (shown === "cameras") showCameras();
  else showCamera(camera);
}

function read() {
  fetch("status", { cache: "no-store" })
    .then((reply) => {
      if (!reply.ok) throw new Error("status " + reply.status);
      return reply.json();
    })
    .then((status) => {
      documentNow = status;
      $("#reach").textContent = "";
      showScreen();
    })
    .catch(() => {
      $("#reach").textContent = "Can't reach Visor: retrying";
    });
}

function start() {
  $("#set").addEventListener("click", () => {
    const camera = cameraNamed(shown);
    if (camera) post(camera.set, $("#brightness .answer"));
  });
  $("#forget").addEventListener("click", () => {
    const camera = cameraNamed(shown);
    if (camera) post(camera.forget, $("#focus .answer"));
  });
  const form = hand();
  form.addEventListener("submit", applyByHand);
  form.exposureSlider.addEventListener("input", () => {
    form.exposure.value = (exposureOfSlider(form) / 1000).toFixed(1);
  });
  form.gainSlider.addEventListener("input", () => { form.gain.value = form.gainSlider.value; });
  form.exposureSlider.addEventListener("change", applyByHand);
  form.gainSlider.addEventListener("change", applyByHand);
  window.addEventListener("hashchange", showScreen);
  window.addEventListener("resize", layOut);
  read();
  setInterval(read, READ_EVERY_MS);
}

start();
