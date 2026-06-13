(function () {
  "use strict";

  const VERSION = "2026-06-13-vision-floor-overlay";
  const STALE_MS = 2400;
  const BASE_PICK_MM = { x: 35, y: 175, z: 0 };
  const SCENE_SCALE = { x: 1 / 180, z: 1 / 178, h: 1 / 220 };
  const LABELS = {
    BOLT: "볼트",
    NUT: "너트",
    WASHER: "와셔",
    PART: "부품",
    OBJECT: "물체",
    NONE: "대기"
  };
  const COLORS = {
    cyan: 0x20d5e8,
    green: 0x4ade80,
    yellow: 0xf7c948,
    blue: 0x60a5fa,
    red: 0xff5b5b,
    white: 0xe6f7ff,
    metal: 0xb9c7d3
  };

  let externalDetection = null;
  let externalAt = 0;
  let lastDetection = null;
  let demoStart = 0;
  let overlay = null;
  let hudEl = null;

  const params = new URLSearchParams(location.search);
  const demoEnabled = /^(1|true|yes|on)$/i.test(params.get("objectOverlayDemo") || "") || params.has("visionDemo");

  function dashState() {
    try {
      if (typeof state !== "undefined") return state;
    } catch (err) {}
    return null;
  }

  function dashThreeRobot() {
    try {
      if (typeof threeRobot !== "undefined") return threeRobot;
    } catch (err) {}
    return null;
  }

  function finite(v) {
    return Number.isFinite(Number(v));
  }

  function num(v, fallback = 0) {
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
  }

  function clamp(v, min, max) {
    return Math.max(min, Math.min(max, v));
  }

  function firstFinite(obj, keys) {
    if (!obj) return null;
    for (const key of keys) {
      if (finite(obj[key])) return Number(obj[key]);
    }
    return null;
  }

  function bestObject(payload) {
    if (!payload) return null;
    const list = Array.isArray(payload.objects) ? payload.objects
      : Array.isArray(payload.detections) ? payload.detections
      : Array.isArray(payload.results) ? payload.results
      : null;
    if (!list || !list.length) return null;
    return list.reduce((best, item) => {
      const a = num(best?.confidence ?? best?.score, 0);
      const b = num(item?.confidence ?? item?.score, 0);
      return b > a ? item : best;
    }, list[0]);
  }

  function normalizeTarget(value) {
    const raw = String(value || "").trim();
    if (!raw) return "NONE";
    const upper = raw.toUpperCase().replace(/[^A-Z0-9_]/g, "_");
    if (upper.includes("BOLT") || raw.includes("볼트")) return "BOLT";
    if (upper.includes("NUT") || raw.includes("너트")) return "NUT";
    if (upper.includes("WASHER") || raw.includes("와셔")) return "WASHER";
    if (upper === "NONE" || upper === "NO_OBJECT" || upper === "WAIT") return "NONE";
    return upper || "OBJECT";
  }

  function labelOf(target) {
    const key = normalizeTarget(target);
    return LABELS[key] || target || LABELS.OBJECT;
  }

  function coordinateFromObject(obj) {
    if (!obj) return null;
    const directX = firstFinite(obj, ["x_mm", "world_x_mm", "arm_x_mm", "table_x_mm", "pick_x_mm", "target_x_mm"]);
    const directY = firstFinite(obj, ["y_mm", "world_y_mm", "arm_y_mm", "table_y_mm", "pick_y_mm", "target_y_mm"]);
    const directZ = firstFinite(obj, ["z_mm", "world_z_mm", "arm_z_mm", "height_mm", "target_z_mm"]);
    if (directX != null && directY != null) return { x: directX, y: directY, z: directZ || 0, source: "direct-mm" };

    const sources = [
      obj.position_mm, obj.world_mm, obj.arm_mm, obj.table_mm, obj.pick_mm,
      obj.target_mm, obj.center_mm, obj.position, obj.world, obj.point
    ];
    for (const src of sources) {
      if (!src) continue;
      if (Array.isArray(src) && src.length >= 2 && finite(src[0]) && finite(src[1])) {
        const unit = String(obj.unit || src.unit || "").toLowerCase();
        const mul = unit === "m" || unit === "meter" || unit === "meters" ? 1000 : 1;
        return { x: Number(src[0]) * mul, y: Number(src[1]) * mul, z: num(src[2], 0) * mul, source: "array" };
      }
      const x = firstFinite(src, ["x_mm", "world_x_mm", "arm_x_mm", "table_x_mm", "x"]);
      const y = firstFinite(src, ["y_mm", "world_y_mm", "arm_y_mm", "table_y_mm", "y"]);
      const z = firstFinite(src, ["z_mm", "world_z_mm", "arm_z_mm", "z", "height"]);
      if (x != null && y != null) {
        const unit = String(src.unit || obj.unit || "").toLowerCase();
        const mul = unit === "m" || unit === "meter" || unit === "meters" ? 1000 : 1;
        return { x: x * mul, y: y * mul, z: (z || 0) * mul, source: "object" };
      }
    }
    return null;
  }

  function normalizeDetection(payload, source) {
    if (!payload) return null;
    const picked = bestObject(payload);
    const obj = Object.assign({}, payload, picked || {}, payload.object || {}, payload.detected_object || {});
    const target = normalizeTarget(obj.target ?? obj.label ?? obj.class_name ?? obj.class ?? obj.name ?? payload.target);
    const confidence = clamp(num(obj.confidence ?? obj.score ?? payload.confidence, 0), 0, 1);
    const correction = Object.assign({}, payload.correction || {}, obj.correction || {});
    let point = coordinateFromObject(obj) || coordinateFromObject(payload);
    if (!point && (target !== "NONE" || confidence > 0.05)) {
      point = {
        x: BASE_PICK_MM.x + num(correction.dx_mm ?? correction.x_mm ?? correction.dx, 0),
        y: BASE_PICK_MM.y + num(correction.dy_mm ?? correction.y_mm ?? correction.dy, 0),
        z: BASE_PICK_MM.z + num(correction.dz_mm ?? correction.z_mm ?? correction.dz, 0),
        source: "scan-correction"
      };
    }
    if (!point) return null;
    return {
      target,
      label: labelOf(target),
      confidence,
      stableFrames: num(obj.stable_frames ?? payload.stable_frames, 0),
      bbox: obj.bbox || payload.bbox || null,
      correction,
      xMm: point.x,
      yMm: point.y,
      zMm: point.z,
      pointSource: point.source,
      source,
      at: performance.now(),
      raw: payload
    };
  }

  function readDetection(now) {
    if (externalDetection && now - externalAt < STALE_MS) return Object.assign({}, externalDetection, { ageMs: now - externalAt });
    if (demoEnabled) return demoDetection(now);
    const s = dashState();
    const d = normalizeDetection(s?.vision, "WebSocket vision");
    if (!d || d.target === "NONE" || d.confidence <= 0.05) return null;
    return d;
  }

  function demoDetection(now) {
    if (!demoStart) demoStart = now;
    const t = (now - demoStart) / 1000;
    const keys = ["BOLT", "NUT", "WASHER"];
    const target = keys[Math.floor(t / 6) % keys.length];
    return {
      target,
      label: labelOf(target),
      confidence: 0.92 + Math.sin(t * 1.8) * 0.04,
      stableFrames: 5,
      correction: { dx_mm: Math.sin(t * 0.8) * 18, dy_mm: Math.cos(t * 0.7) * 12 },
      xMm: BASE_PICK_MM.x + Math.sin(t * 0.8) * 44,
      yMm: BASE_PICK_MM.y + Math.cos(t * 0.7) * 30,
      zMm: 0,
      pointSource: "demo",
      source: "VISION DEMO",
      at: now,
      ageMs: 0,
      raw: null
    };
  }

  function mmToScene(xMm, yMm, zMm) {
    return {
      x: clamp(num(xMm, 0) * SCENE_SCALE.x, -2.6, 2.6),
      y: 0.04 + clamp(num(zMm, 0) * SCENE_SCALE.h, 0, 1.6),
      z: clamp(num(yMm, 0) * SCENE_SCALE.z, -1.8, 2.25)
    };
  }

  function gripperScenePoint(T, r) {
    if (r?.gripperMesh?.getWorldPosition) {
      const p = new T.Vector3();
      r.gripperMesh.getWorldPosition(p);
      if (Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z)) return p;
    }
    const ee = dashState()?.robot?.end_effector;
    if (ee && finite(ee.x) && finite(ee.y)) {
      const p = mmToScene(ee.x, ee.y, ee.z);
      return new T.Vector3(p.x, clamp(p.y + 0.16, 0.18, 2.4), p.z);
    }
    return new T.Vector3(0, 0.45, 0.3);
  }

  function makeTextSprite(T, text, accent) {
    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    canvas.width = 512;
    canvas.height = 160;
    const texture = new T.CanvasTexture(canvas);
    if (T.SRGBColorSpace) texture.colorSpace = T.SRGBColorSpace;
    const sprite = new T.Sprite(new T.SpriteMaterial({ map: texture, transparent: true, depthTest: false, depthWrite: false }));
    sprite.renderOrder = 50;
    sprite.scale.set(0.98, 0.31, 1);
    sprite.userData = { canvas, ctx, texture, text: "", accent: "" };
    updateTextSprite(sprite, text, accent);
    return sprite;
  }

  function updateTextSprite(sprite, text, accent) {
    if (!sprite?.userData?.ctx || (sprite.userData.text === text && sprite.userData.accent === accent)) return;
    const { canvas, ctx, texture } = sprite.userData;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = "rgba(5,12,20,.84)";
    roundedRect(ctx, 8, 8, canvas.width - 16, canvas.height - 16, 18);
    ctx.fill();
    ctx.strokeStyle = accent;
    ctx.lineWidth = 5;
    roundedRect(ctx, 10, 10, canvas.width - 20, canvas.height - 20, 16);
    ctx.stroke();
    const lines = String(text).split("\n").slice(0, 3);
    ctx.textAlign = "center";
    ctx.fillStyle = "#e6f7ff";
    ctx.font = "700 34px Malgun Gothic, Segoe UI, sans-serif";
    ctx.fillText(lines[0] || "", canvas.width / 2, 58);
    ctx.fillStyle = accent;
    ctx.font = "700 26px Consolas, Malgun Gothic, monospace";
    ctx.fillText(lines[1] || "", canvas.width / 2, 101);
    ctx.fillStyle = "rgba(230,247,255,.72)";
    ctx.font = "600 20px Consolas, Malgun Gothic, monospace";
    ctx.fillText(lines[2] || "", canvas.width / 2, 132);
    sprite.userData.text = text;
    sprite.userData.accent = accent;
    texture.needsUpdate = true;
  }

  function roundedRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function material(T, color, opacity, extra = {}) {
    return new T.MeshBasicMaterial(Object.assign({
      color,
      transparent: opacity < 1,
      opacity,
      side: T.DoubleSide,
      depthWrite: false
    }, extra));
  }

  function makePartMeshes(T) {
    const metal = new T.MeshStandardMaterial({ color: COLORS.metal, metalness: 0.38, roughness: 0.44 });
    const accent = new T.MeshStandardMaterial({ color: COLORS.yellow, metalness: 0.18, roughness: 0.5 });
    const group = new T.Group();
    const makeBolt = () => {
      const g = new T.Group();
      const shaft = new T.Mesh(new T.CylinderGeometry(0.032, 0.032, 0.34, 24), metal);
      const head = new T.Mesh(new T.CylinderGeometry(0.09, 0.09, 0.058, 6), metal);
      shaft.rotation.z = Math.PI / 2;
      head.rotation.z = Math.PI / 2;
      head.position.x = -0.18;
      shaft.castShadow = head.castShadow = true;
      g.add(shaft, head);
      return g;
    };
    const makeNut = () => {
      const g = new T.Group();
      const nut = new T.Mesh(new T.TorusGeometry(0.105, 0.033, 18, 6), metal);
      nut.rotation.x = Math.PI / 2;
      nut.castShadow = true;
      g.add(nut);
      return g;
    };
    const makeWasher = () => {
      const g = new T.Group();
      const washer = new T.Mesh(new T.TorusGeometry(0.13, 0.022, 20, 48), metal);
      washer.rotation.x = Math.PI / 2;
      washer.castShadow = true;
      g.add(washer);
      return g;
    };
    const makeObject = () => {
      const g = new T.Group();
      const puck = new T.Mesh(new T.CylinderGeometry(0.105, 0.105, 0.07, 36), accent);
      puck.position.y = 0.035;
      puck.castShadow = true;
      g.add(puck);
      return g;
    };
    group.userData.parts = {
      BOLT: makeBolt(),
      NUT: makeNut(),
      WASHER: makeWasher(),
      OBJECT: makeObject()
    };
    Object.values(group.userData.parts).forEach(part => {
      part.position.y = 0.09;
      group.add(part);
    });
    return group;
  }

  function ensureThreeOverlay() {
    const T = window.THREE;
    const r = dashThreeRobot();
    if (!T || !r?.scene) return null;
    if (overlay?.scene === r.scene) return overlay;
    if (overlay?.root?.parent) overlay.root.parent.remove(overlay.root);

    const root = new T.Group();
    root.name = "sambo-object-vision-floor-overlay";
    root.renderOrder = 30;
    const target = new T.Group();
    root.add(target);

    const disk = new T.Mesh(new T.CircleGeometry(0.16, 80), material(T, COLORS.cyan, 0.16));
    disk.rotation.x = -Math.PI / 2;
    disk.position.y = 0.028;
    target.add(disk);

    const ring = new T.Mesh(new T.RingGeometry(0.18, 0.215, 96), material(T, COLORS.cyan, 0.95));
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.034;
    target.add(ring);

    const outerRing = new T.Mesh(new T.RingGeometry(0.34, 0.355, 128), material(T, COLORS.cyan, 0.42));
    outerRing.rotation.x = -Math.PI / 2;
    outerRing.position.y = 0.038;
    target.add(outerRing);

    const beam = new T.Mesh(new T.CylinderGeometry(0.009, 0.009, 0.78, 18), material(T, COLORS.cyan, 0.36));
    beam.position.y = 0.42;
    target.add(beam);

    const vertical = new T.Mesh(new T.RingGeometry(0.11, 0.116, 64), material(T, COLORS.green, 0.74));
    vertical.position.y = 0.29;
    vertical.rotation.y = Math.PI / 2;
    target.add(vertical);

    const parts = makePartMeshes(T);
    target.add(parts);

    const label = makeTextSprite(T, "VISION LOCK\nX +0 Y +0", "#20d5e8");
    label.position.set(0, 0.64, 0.03);
    target.add(label);

    const lineGeometry = new T.BufferGeometry().setFromPoints([new T.Vector3(), new T.Vector3()]);
    const approachLine = new T.Line(lineGeometry, new T.LineBasicMaterial({
      color: COLORS.green,
      transparent: true,
      opacity: 0.72,
      depthWrite: false
    }));
    approachLine.renderOrder = 29;
    root.add(approachLine);

    r.scene.add(root);
    overlay = { T, scene: r.scene, robot: r, root, target, disk, ring, outerRing, beam, vertical, parts, label, approachLine };
    return overlay;
  }

  function colorForTarget(target) {
    const key = normalizeTarget(target);
    if (key === "BOLT") return "#4ade80";
    if (key === "NUT") return "#60a5fa";
    if (key === "WASHER") return "#f7c948";
    return "#20d5e8";
  }

  function showPart(parts, target) {
    const key = normalizeTarget(target);
    const items = parts?.userData?.parts || {};
    Object.entries(items).forEach(([name, mesh]) => {
      mesh.visible = name === key || (key !== "BOLT" && key !== "NUT" && key !== "WASHER" && name === "OBJECT");
    });
  }

  function updateThree(detection, now) {
    const o = ensureThreeOverlay();
    if (!o) return;
    o.root.visible = !!detection;
    if (!detection) return;

    const p = mmToScene(detection.xMm, detection.yMm, detection.zMm);
    o.target.position.set(p.x, 0, p.z);
    const pulse = 1 + Math.sin(now * 0.008) * 0.055;
    const slowPulse = 1.08 + Math.sin(now * 0.003) * 0.08;
    o.ring.scale.setScalar(pulse);
    o.outerRing.scale.setScalar(slowPulse);
    o.disk.material.opacity = 0.13 + Math.sin(now * 0.006) * 0.035;
    o.beam.material.opacity = 0.25 + Math.sin(now * 0.005) * 0.12;
    o.vertical.rotation.z = now * 0.0025;
    o.parts.rotation.y = now * 0.0012;
    showPart(o.parts, detection.target);

    const accent = colorForTarget(detection.target);
    const label = `${detection.label} ${(detection.confidence * 100).toFixed(0)}%`;
    const coord = `X ${detection.xMm >= 0 ? "+" : ""}${detection.xMm.toFixed(1)}  Y ${detection.yMm >= 0 ? "+" : ""}${detection.yMm.toFixed(1)} mm`;
    const meta = `${detection.source} - ${detection.stableFrames || 0}/3 frames`;
    updateTextSprite(o.label, `${label}\n${coord}\n${meta}`, accent);

    const start = gripperScenePoint(o.T, o.robot);
    const end = new o.T.Vector3(p.x, 0.1, p.z);
    o.approachLine.geometry.setFromPoints([start, end]);
    o.approachLine.material.opacity = detection.confidence >= 0.7 ? 0.78 : 0.38;
  }

  function injectStyle() {
    if (document.getElementById("samboObjectOverlayStyle")) return;
    const style = document.createElement("style");
    style.id = "samboObjectOverlayStyle";
    style.textContent = `
.sambo-vision-hud{position:absolute;left:16px;bottom:16px;z-index:8;width:min(320px,calc(100% - 32px));pointer-events:none;color:#e6f7ff;font-family:Malgun Gothic,Segoe UI,sans-serif}
.sambo-vision-hud .box{border:1px solid rgba(32,213,232,.78);background:linear-gradient(180deg,rgba(5,12,20,.86),rgba(8,20,34,.78));box-shadow:0 0 22px rgba(32,213,232,.18);border-radius:8px;padding:10px 12px}
.sambo-vision-hud .top{display:flex;align-items:center;justify-content:space-between;gap:10px;font-size:11px;font-weight:800;letter-spacing:0;color:#20d5e8}
.sambo-vision-hud .lock{display:flex;align-items:center;gap:7px;white-space:nowrap}
.sambo-vision-hud .dot{width:8px;height:8px;border-radius:50%;background:#4ade80;box-shadow:0 0 14px #4ade80}
.sambo-vision-hud.search .dot{background:#f7c948;box-shadow:0 0 14px #f7c948}
.sambo-vision-hud .object{margin-top:6px;font-size:19px;font-weight:900;line-height:1.15}
.sambo-vision-hud .coords{margin-top:5px;display:grid;grid-template-columns:1fr 1fr;gap:6px;font:700 12px Consolas,monospace;color:#cdeaff}
.sambo-vision-hud .meta{margin-top:6px;font:700 10px Consolas,monospace;color:rgba(230,247,255,.68);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.sambo-vision-hud .bar{height:4px;margin-top:8px;background:rgba(148,163,184,.24);border-radius:999px;overflow:hidden}
.sambo-vision-hud .bar span{display:block;height:100%;background:linear-gradient(90deg,#20d5e8,#4ade80);border-radius:inherit}
`;
    document.head.appendChild(style);
  }

  function ensureHud() {
    injectStyle();
    const canvas = document.getElementById("robot3dCanvas");
    const wrap = canvas?.closest(".media-wrap") || canvas?.parentElement;
    if (!wrap) return null;
    if (getComputedStyle(wrap).position === "static") wrap.style.position = "relative";
    if (!hudEl || !wrap.contains(hudEl)) {
      hudEl = document.createElement("div");
      hudEl.id = "samboVisionHud";
      hudEl.className = "sambo-vision-hud search";
      wrap.appendChild(hudEl);
    }
    return hudEl;
  }

  function updateHud(detection) {
    const el = ensureHud();
    if (!el) return;
    if (!detection) {
      el.className = "sambo-vision-hud search";
      el.innerHTML = `<div class="box"><div class="top"><span class="lock"><i class="dot"></i>VISION SEARCH</span><span>SCAN FLOOR</span></div><div class="object">물체 대기</div><div class="coords"><span>X -- mm</span><span>Y -- mm</span></div><div class="meta">WebSocket vision 또는 overlay API 대기</div><div class="bar"><span style="width:0%"></span></div></div>`;
      return;
    }
    el.className = "sambo-vision-hud lock";
    const conf = clamp(detection.confidence * 100, 0, 100);
    const age = detection.ageMs != null ? `${Math.round(detection.ageMs)} ms` : "live";
    el.innerHTML = `<div class="box"><div class="top"><span class="lock"><i class="dot"></i>VISION LOCK</span><span>${detection.source}</span></div><div class="object">${detection.label} ${conf.toFixed(0)}%</div><div class="coords"><span>X ${detection.xMm >= 0 ? "+" : ""}${detection.xMm.toFixed(1)} mm</span><span>Y ${detection.yMm >= 0 ? "+" : ""}${detection.yMm.toFixed(1)} mm</span></div><div class="meta">stable ${detection.stableFrames || 0}/3 - ${age} - ${detection.pointSource}</div><div class="bar"><span style="width:${conf.toFixed(0)}%"></span></div></div>`;
  }

  function tick(now) {
    const detection = readDetection(now);
    lastDetection = detection;
    updateThree(detection, now);
    updateHud(detection);
    requestAnimationFrame(tick);
  }

  function updateDetection(payload) {
    if (!payload) {
      externalDetection = null;
      externalAt = 0;
      return null;
    }
    externalDetection = normalizeDetection(payload, "overlay API");
    externalAt = performance.now();
    return externalDetection;
  }

  window.samboObjectOverlay = {
    version: VERSION,
    updateDetection,
    clear: () => updateDetection(null),
    simulate: (payload = {}) => updateDetection(Object.assign({
      target: "BOLT",
      confidence: 0.96,
      stable_frames: 5,
      x_mm: BASE_PICK_MM.x,
      y_mm: BASE_PICK_MM.y,
      z_mm: 0
    }, payload)),
    getStatus: () => ({ version: VERSION, demoEnabled, lastDetection, externalDetection })
  };

  window.addEventListener("sambo:vision-object", event => updateDetection(event.detail));
  requestAnimationFrame(tick);
})();
