// DOM wiring and pointer interaction. All maths lives in geometry/kinematics.

import * as G from './geometry.js';
import * as M from './model.js';
import * as K from './kinematics.js';
import { derivePoles, derivedPoles, poleName } from './kennedy.js';
import { render } from './render.js';
import { createTrajectory, describeMode, advanceParams, paramFromValue, valueFromParam, displayRange, FREE_TRAVEL } from './motion.js';
import { encodeSetup, parseHash, hashWithSetup, isViewer, optionFlag, VIEW_FLAGS, DEFAULT_FLAGS, DEFAULT_SCALE } from './serialize.js';

const $ = (id) => document.getElementById(id);
const PICK_PX = 10; // hit radius for supports / poles (screen px)
const LINE_PX = 7; // hit distance for bodies and beams (screen px)
const CLICK_PX = 4; // pointer travel below this is a click, not a drag
const PLAY_SPEED = 0.04; // mode parameter (length units) advanced per animation frame; ranges longer than 24 play up to 4x faster
const FREE_RANGE = { kind: 'unbounded', min: -FREE_TRAVEL, max: FREE_TRAVEL }; // shown until the real ranges are known

export function startApp(params, initial = {}) {
  const canvas = $('view');
  const ctx = canvas.getContext('2d');
  // Viewer: read-only presentation for iframes (embed=1 or view=1); edit=1 keeps the full editor inside an iframe.
  const viewer = isViewer(params);
  const zoomEnabled = optionFlag(params, 'zoom') !== false; // viewer only: zoom=0 freezes the view (no wheel zoom, no pan)

  const S = {
    scene: M.createScene(),
    sys: null,
    q: [],
    analysis: null,
    dq: null,
    items: [],
    intersections: [],
    poles: new Map(),
    tool: 'select',
    view: { cx: 0, cy: 2, scale: 40 },
    size: { w: 300, h: 300, dpr: 1 },
    opts: { snap: true, ...DEFAULT_FLAGS, scale: DEFAULT_SCALE, mode: 0 },
    sel: null,
    poleSel: new Set(),
    drag: null,
    preview: null,
    chain: null, // open polyline being drawn: { target: body id | null, pts: [...] }
    // Simulation: `traj` is the finite-motion trajectory rebased at the pose where the user last left it.
    // params/dir/active/locked/ranges are per mode (see motion.js): params are display values (the slider
    // positions), offsets[i] the display value at the rebase pose, ranges[i] the display range (min, max, kind).
    // traj = null means "rebase when needed"; stale = another mode moved, so the ranges are re-derived at the
    // current pose (staleTimer) before they are relied on.
    sim: { on: false, playing: false, traj: null, params: [], offsets: [], dir: [], active: [], locked: [], ranges: null, angleRefs: [], rebaseTimer: 0, stale: false, staleTimer: 0 },
  };
  const share = { timer: 0, seq: 0, last: '', live: false }; // address-bar sync state (see "share link" below)

  // ------------------------------------------------------------ model refresh

  /** Scene geometry/supports changed: drop any displacement and rebuild everything. */
  function edited() {
    S.sys = K.buildSystem(S.scene);
    S.q = K.zeroState(S.sys);
    dropTrajectory();
    if (S.sel && !findSelected()) S.sel = null;
    refresh();
    renderProps();
  }

  /** Pose changed (or mode/selection changed): redo analysis-derived data and redraw. */
  function refresh() {
    S.analysis = K.analyze(S.sys, S.q);
    S.opts.mode = Math.min(S.opts.mode, Math.max(0, S.analysis.nullity - 1));
    S.dq = S.analysis.nullity ? K.normalizeMode(S.sys, S.q, S.analysis.modes[S.opts.mode]) : null;
    S.items = K.poleItems(S.sys, S.q, S.scene);
    const ids = new Set(S.items.map((it) => it.id));
    for (const id of [...S.poleSel]) if (!ids.has(id)) S.poleSel.delete(id);
    S.intersections = K.selectedIntersections(S.items, S.poleSel);
    S.poles = derivePoles(S.items, S.sys.n);
    updatePanels();
    updateMotionUI();
    draw();
    scheduleHashUpdate();
  }

  function findSelected() {
    if (!S.sel) return null;
    return S.sel.kind === 'body'
      ? S.scene.bodies.find((b) => b.id === S.sel.id)
      : S.scene.supports.find((s) => s.id === S.sel.id);
  }

  // ----------------------------------------------------------------- drawing

  let drawQueued = false;
  function draw() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => {
      drawQueued = false;
      render(ctx, {
        ...S.size, view: S.view, scene: S.scene, sys: S.sys, q: S.q, dq: S.dq,
        items: S.items, intersections: S.intersections, poles: S.poles, opts: S.opts,
        sel: S.sel, poleSel: S.poleSel, preview: S.preview,
      });
    });
  }

  function resize() {
    const r = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    S.size = { w: Math.max(50, r.width), h: Math.max(50, r.height), dpr };
    canvas.width = Math.round(S.size.w * dpr);
    canvas.height = Math.round(S.size.h * dpr);
    draw();
  }

  function fitView() {
    const pts = S.scene.bodies.flatMap((b) => b.pts).concat(S.scene.supports.map((s) => s.p));
    if (!pts.length) {
      S.view = { cx: 0, cy: 2, scale: 40 };
    } else {
      const xs = pts.map((p) => p.x);
      const ys = pts.map((p) => p.y);
      const wx = Math.max(...xs) - Math.min(...xs) + 4;
      const wy = Math.max(...ys) - Math.min(...ys) + 4;
      S.view = {
        cx: (Math.max(...xs) + Math.min(...xs)) / 2,
        cy: (Math.max(...ys) + Math.min(...ys)) / 2,
        scale: Math.max(8, Math.min(S.size.w / wx, S.size.h / wy, 120)),
      };
    }
    draw();
  }

  // ------------------------------------------------------------- coordinates

  const toWorld = (sp) => ({
    x: (sp.x - S.size.w / 2) / S.view.scale + S.view.cx,
    y: (S.size.h / 2 - sp.y) / S.view.scale + S.view.cy,
  });
  const toScreen = (p) => ({
    x: (p.x - S.view.cx) * S.view.scale + S.size.w / 2,
    y: S.size.h / 2 - (p.y - S.view.cy) * S.view.scale,
  });
  const snap = (p) => {
    if (!S.opts.snap) return p;
    const st = G.gridStep(S.view.scale);
    return { x: G.snapValue(p.x, st), y: G.snapValue(p.y, st) };
  };
  function eventPos(e) {
    const r = canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }

  // ----------------------------------------------------------------- hit tests

  const currentPoints = (i) => S.sys.bodies[i].pts.map((p) => K.worldPoint(S.q, i, p));

  /** Topmost body index under world point w, or -1. */
  function hitBody(w) {
    const tol = LINE_PX / S.view.scale;
    for (let i = S.scene.bodies.length - 1; i >= 0; i--) {
      const b = S.scene.bodies[i];
      if (M.pointHitsShape(b.kind, currentPoints(i), w, tol, b.segs)) return i;
    }
    return -1;
  }

  function hitSupport(sp) {
    for (let i = S.scene.supports.length - 1; i >= 0; i--) {
      const s = S.scene.supports[i];
      if (G.dist(toScreen(K.supportPoint(S.sys, S.q, s)), sp) <= PICK_PX) return s;
    }
    return null;
  }

  /** Pole item (visible beam or pole marker) under screen point sp. */
  function hitPoleItem(sp) {
    for (const it of S.items) {
      const P = toScreen(it.point);
      if (it.kind === 'point') {
        if (G.dist(P, sp) <= PICK_PX + 2) return it;
      } else {
        const d = { x: it.dir.x, y: -it.dir.y };
        if (Math.abs(G.cross(d, G.sub(sp, P))) <= LINE_PX) return it;
      }
    }
    return null;
  }

  // ------------------------------------------------------------------ messages

  let msgTimer = 0;
  function message(text, ms = 3500) {
    const el = $('msg');
    el.textContent = text;
    el.style.display = text ? 'block' : 'none';
    clearTimeout(msgTimer);
    if (text && ms) msgTimer = setTimeout(() => message(''), ms);
  }

  // ------------------------------------------------------------------ pointer

  canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  canvas.addEventListener('wheel', (e) => {
    if (viewer && !zoomEnabled) return; // let the embedding page scroll
    e.preventDefault();
    const sp = eventPos(e);
    const before = toWorld(sp);
    S.view.scale = Math.min(300, Math.max(4, S.view.scale * Math.exp(-e.deltaY * 0.0015)));
    const after = toWorld(sp);
    S.view.cx += before.x - after.x;
    S.view.cy += before.y - after.y;
    draw();
  }, { passive: false });

  canvas.addEventListener('pointerdown', (e) => {
    if (S.drag) return; // ignore additional touches
    const sp = eventPos(e);
    if (viewer) {
      // Read-only: the only pointer action is panning the view.
      if (!zoomEnabled) return;
      canvas.setPointerCapture(e.pointerId);
      S.drag = { kind: 'pan', last: sp };
      return;
    }
    canvas.setPointerCapture(e.pointerId);
    const w = toWorld(sp);
    if (e.button === 1 || e.button === 2) {
      S.drag = { kind: 'pan', last: sp };
      return;
    }
    switch (S.tool) {
      case 'select': return downSelect(sp, w);
      case 'line': case 'addseg': downLine(sp, w); break;
      case 'pin': case 'roller': case 'joint': placeSupport(S.tool, snap(w)); break;
      case 'poles': {
        const it = hitPoleItem(sp);
        if (it) togglePole(it.id);
        break;
      }
      case 'delete': deleteAt(sp, w); break;
    }
    draw();
  });

  canvas.addEventListener('pointermove', (e) => {
    const d = S.drag;
    if (!d) {
      if (S.chain) {
        S.preview = chainPreview(snap(toWorld(eventPos(e))));
        draw();
      }
      return;
    }
    const sp = eventPos(e);
    const w = toWorld(sp);
    switch (d.kind) {
      case 'chainClick': return;
      case 'pan':
        S.view.cx -= (sp.x - d.last.x) / S.view.scale;
        S.view.cy += (sp.y - d.last.y) / S.view.scale;
        d.last = sp;
        break;
      case 'line': S.preview = { kind: 'line', a: d.a, b: snap(w) }; break;
      case 'moveSupport': {
        const s = S.scene.supports.find((x) => x.id === d.id);
        s.p = snap(w);
        edited();
        return;
      }
      case 'moveBody': moveBody(d, snap(w)); return;
      case 'simDrag':
        S.q = K.dragTo(S.sys, S.q, d.body, d.pRef, w);
        dropTrajectory();
        refresh();
        return;
    }
    draw();
  });

  function endDrag(e) {
    const d = S.drag;
    if (!d) return;
    S.drag = null;
    S.preview = S.chain ? chainPreview(snap(toWorld(eventPos(e)))) : null;
    if (d.kind === 'line') {
      const sp = eventPos(e);
      const b = snap(toWorld(sp));
      if (G.dist(sp, d.sp) < CLICK_PX) {
        // A click, not a drag: start a multi-segment chain from the press point.
        S.chain = { target: d.target, pts: [d.a] };
        S.preview = chainPreview(d.a);
        message('Click further points; double-click, Enter or Esc finishes the body.');
      } else if (G.dist(d.a, b) > 1e-6) {
        commitChain({ target: d.target, pts: [d.a, b] });
      }
    } else if (d.kind === 'chainClick') {
      clickChain(eventPos(e));
    } else if (d.kind === 'pan' || d.kind === 'simDrag') {
      /* nothing to commit */
    }
    draw();
  }
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', () => {
    S.drag = null;
    S.preview = null;
    draw();
  });

  // -- line / add-segment tools: drag = one segment; click = start a chain of connected segments

  /** Start point for "Add segment": a node of the clicked rod, else a point on it. Null (with message) if none. */
  function attachStart(w) {
    if (S.q.some((v) => v !== 0)) return void message('Reset the simulated pose before adding a segment.');
    const bi = hitBody(w);
    if (bi < 0) return void message('Click a line body to add a segment to it.');
    const body = S.scene.bodies[bi];
    const tol = LINE_PX / S.view.scale;
    const node = body.pts.find((p) => G.dist(p, w) <= tol);
    const snapped = snap(w);
    const onBody = G.dist(M.nearestPointOnBody(body, snapped), snapped) < 1e-6;
    return { id: body.id, p: node ? { ...node } : onBody ? snapped : M.nearestPointOnBody(body, w) };
  }

  function downLine(sp, w) {
    if (S.chain) {
      S.drag = { kind: 'chainClick' };
      return;
    }
    let a = snap(w);
    let target = null;
    if (S.tool === 'addseg') {
      const start = attachStart(w);
      if (!start) return;
      a = start.p;
      target = start.id;
      selectItem({ kind: 'body', id: target });
    }
    S.drag = { kind: 'line', a, sp, target };
    S.preview = { kind: 'line', a, b: a };
  }

  const chainPreview = (cursor) => ({ kind: 'chain', pts: [...S.chain.pts, cursor] });

  /** A click while a chain is open: add a point, or finish when it repeats the last point (double-click). */
  function clickChain(sp) {
    const last = S.chain.pts[S.chain.pts.length - 1];
    const next = snap(toWorld(sp));
    if (G.dist(next, last) < 1e-9 || G.dist(toScreen(last), sp) < CLICK_PX) return finishChain();
    S.chain.pts.push(next);
    S.preview = chainPreview(S.chain.pts[S.chain.pts.length - 1]);
  }

  function finishChain() {
    const chain = S.chain;
    S.chain = null;
    S.preview = null;
    if (chain && chain.pts.length >= 2) commitChain(chain);
    draw();
  }

  /** Creates a body from the chain, or (chain.target) adds it to that body, merging a rod it ends on. */
  function commitChain(chain) {
    const pts = chain.pts.slice();
    let id;
    if (chain.target != null) {
      const end = pts[pts.length - 1];
      const tol = LINE_PX / S.view.scale;
      const other = S.scene.bodies.find((b) => b.id !== chain.target && b.kind === 'rod' && M.pointHitsShape(b.kind, b.pts, end, tol, b.segs));
      if (other) pts[pts.length - 1] = M.nearestPointOnBody(other, end);
      if (!M.extendBody(S.scene, chain.target, pts)) return;
      if (other) M.mergeBodies(S.scene, chain.target, other.id);
      id = chain.target;
    } else {
      const body = M.addPolyline(S.scene, pts);
      if (!body) return;
      id = body.id;
    }
    S.sel = { kind: 'body', id };
    edited();
  }

  function downSelect(sp, w) {
    const sup = hitSupport(sp);
    const bi = hitBody(w);
    if (S.sim.on && bi >= 0 && !sup) {
      S.drag = { kind: 'simDrag', body: bi, pRef: K.bodyPoint(S.q, bi, w) };
      selectItem({ kind: 'body', id: S.scene.bodies[bi].id });
      return;
    }
    if (sup) {
      selectItem({ kind: 'support', id: sup.id });
      if (!S.sim.on) S.drag = { kind: 'moveSupport', id: sup.id };
    } else if (bi >= 0) {
      const body = S.scene.bodies[bi];
      selectItem({ kind: 'body', id: body.id });
      if (!S.sim.on) {
        // Carry pins/rollers that currently sit on this body (their topmost body).
        const carried = S.scene.supports.filter((s) => {
          const info = S.sys.supportInfo.get(s.id);
          return s.type !== 'joint' && info.active && info.body === bi;
        });
        S.drag = {
          kind: 'moveBody', id: body.id, start: snap(w),
          pts: body.pts.map((p) => ({ ...p })),
          supports: carried.map((s) => ({ id: s.id, p: { ...s.p } })),
        };
      }
    } else {
      selectItem(null);
      S.drag = { kind: 'pan', last: sp };
    }
    draw();
  }

  function moveBody(d, now) {
    const dx = now.x - d.start.x;
    const dy = now.y - d.start.y;
    const body = S.scene.bodies.find((b) => b.id === d.id);
    body.pts = d.pts.map((p) => ({ x: p.x + dx, y: p.y + dy }));
    for (const s of d.supports) {
      const target = S.scene.supports.find((x) => x.id === s.id);
      if (target) target.p = { x: s.p.x + dx, y: s.p.y + dy };
    }
    edited();
  }

  function placeSupport(type, p) {
    const n = M.bodiesAtPoint(S.scene, p).length;
    if (type === 'joint' && n < 2) return message('A joint needs at least two bodies at this point.');
    if (type !== 'joint' && n < 1) return message('Place supports on a body.');
    const sup = M.addSupport(S.scene, type, p);
    S.sel = { kind: 'support', id: sup.id };
    edited();
  }

  function deleteAt(sp, w) {
    const sup = hitSupport(sp);
    if (sup) {
      M.removeSupport(S.scene, sup.id);
    } else {
      const bi = hitBody(w);
      if (bi < 0) return;
      M.removeBody(S.scene, S.scene.bodies[bi].id);
    }
    edited();
  }

  function selectItem(sel) {
    S.sel = sel;
    renderProps();
    draw();
  }

  function togglePole(id) {
    if (S.poleSel.has(id)) S.poleSel.delete(id);
    else S.poleSel.add(id);
    refresh();
    renderProps();
  }

  // ---------------------------------------------------------------- toolbar

  function setTool(tool) {
    if (S.chain && tool !== S.tool) finishChain();
    S.tool = tool;
    for (const b of document.querySelectorAll('#tools button')) {
      const on = b.dataset.tool === tool;
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', String(on));
    }
    canvas.style.cursor = tool === 'select' ? 'default' : 'crosshair';
  }
  $('tools').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-tool]');
    if (b) setTool(b.dataset.tool);
  });

  function setSim(on) {
    S.sim.on = on;
    $('chk-sim').checked = on;
    if (!on) setPlaying(false);
  }
  function setPlaying(on) {
    const wasPlaying = S.sim.playing;
    S.sim.playing = on;
    if (!on) S.sim.active = S.sim.active.map(() => false);
    if (wasPlaying && !on) markRangesStale();
    $('btn-play').textContent = on ? 'Pause' : 'Play';
    $('btn-play').classList.toggle('on', on);
    if (on) requestAnimationFrame(tick);
    updateMotionUI();
  }

  // -- finite motion along the independent modes (maths in motion.js)

  let modeDescriptions = [];

  /** The pose was changed by something other than the trajectory: rebase it at the current pose when next needed. */
  function dropTrajectory() {
    clearTimeout(S.sim.staleTimer);
    S.sim.staleTimer = 0;
    S.sim.stale = false;
    S.sim.traj = null;
    S.sim.ranges = null;
  }
  /**
   * Trajectory rebased at the current pose (mode values restart at 0 there); keeps per-mode locks if the mode
   * count is unchanged. `fresh` also re-derives the ranges of a trajectory whose other modes have moved since.
   */
  function ensureTrajectory(fresh = false) {
    const sim = S.sim;
    if (!sim.traj) rebuildTrajectory(null);
    else if (fresh && sim.stale) rebuildTrajectory({ values: sim.params, ranges: sim.ranges });
    return sim.traj;
  }
  /**
   * Builds the trajectory at the current pose and classifies every mode there (the other modes held at
   * this pose). `keep` = { values, ranges } carries over the displayed values and the fixed free-side
   * travel when an existing trajectory is rebased at its current pose.
   */
  function rebuildTrajectory(keep) {
    const sim = S.sim;
    clearTimeout(sim.rebaseTimer);
    clearTimeout(sim.staleTimer);
    sim.rebaseTimer = 0;
    sim.staleTimer = 0;
    sim.stale = false;
    const traj = createTrajectory(S.sys, S.q);
    const k = traj.count;
    const carried = !!keep && keep.values.length === k;
    const infos = traj.modes.map((_, i) => traj.modeRange(i));
    const offsets = carried ? keep.values.slice() : new Array(k).fill(0);
    const angleRefs = infos.map((info, i) => {
      if (!info.angle) return null;
      const before = carried && sim.angleRefs[i];
      if (before && before.body === info.angle.body && before.sign === info.angle.sign) return before;
      return { body: info.angle.body, sign: info.angle.sign, theta: S.q[3 * info.angle.body + 2] };
    });
    sim.traj = traj;
    sim.offsets = offsets;
    sim.angleRefs = angleRefs;
    sim.ranges = infos.map((info, i) => displayRange(info, offsets[i], carried ? keep.ranges[i] : undefined));
    sim.params = offsets.slice();
    if (!carried) {
      sim.dir = new Array(k).fill(1);
      sim.active = new Array(k).fill(false);
    }
    if (sim.locked.length !== k) sim.locked = new Array(k).fill(false);
    S.opts.mode = Math.min(S.opts.mode, Math.max(0, k - 1));
    // Several modes are named after their driver coordinate; a single mode after the body that moves most.
    modeDescriptions = traj.modes.map((m, i) => (traj.drivers[i] ? traj.drivers[i].label : describeMode(S.sys, S.q, m)));
  }
  /** A mode value changed while several modes exist: the ranges of the other modes belong to the old pose. */
  function markRangesStale() {
    const sim = S.sim;
    if (!sim.traj || sim.traj.count < 2) return;
    sim.stale = true;
    clearTimeout(sim.staleTimer);
    sim.staleTimer = setTimeout(() => {
      sim.staleTimer = 0;
      if (sim.traj && sim.stale && !sim.playing) {
        ensureTrajectory(true);
        updateMotionUI();
      }
    }, 150);
  }

  /** Trajectory parameters t for the display values u (per-mode offsets / wrap-around of closed modes). */
  const paramsOf = (u) => u.map((v, i) => paramFromValue(S.sim.ranges[i], S.sim.offsets[i], v));

  /** Moves to the pose for display values u (the pose is a pure function of u); returns false when blocked. */
  function applyValues(u) {
    const sim = S.sim;
    const pose = sim.traj.poseAt(paramsOf(u));
    S.q = pose.q;
    sim.params = pose.blocked ? pose.params.map((t, i) => valueFromParam(sim.ranges[i], sim.offsets[i], t)) : u.slice();
    return !pose.blocked;
  }

  function tick() {
    const sim = S.sim;
    if (!sim.playing) return;
    const traj = ensureTrajectory(true);
    if (!sim.active.some((a, i) => a && !sim.locked[i])) {
      setPlaying(false);
      return;
    }
    const speeds = sim.ranges.map((r) => PLAY_SPEED * Math.min(4, Math.max(1, (r.max - r.min) / 24)));
    const next = advanceParams({ values: sim.params, dir: sim.dir, playing: sim.active, locked: sim.locked, ranges: sim.ranges }, speeds);
    const pose = traj.poseAt(paramsOf(next.values));
    if (pose.blocked) {
      // The combined motion hit a limit that no single mode reaches: turn the playing modes round.
      sim.dir = sim.dir.map((d, i) => (sim.active[i] && !sim.locked[i] ? -d : d));
    } else {
      S.q = pose.q;
      sim.params = next.values;
      sim.dir = next.dir;
    }
    refresh();
    if (sim.playing) requestAnimationFrame(tick);
  }

  /** Sets one mode value (slider scrub): pauses playback and shows the pose for the new values. */
  function scrubMode(i, value) {
    setSim(true);
    setPlaying(false);
    ensureTrajectory();
    const u = S.sim.params.slice();
    u[i] = value;
    if (!applyValues(u)) message('Motion blocked: that combination of modes is out of reach.');
    S.opts.mode = i;
    markRangesStale();
    refresh();
  }

  const KIND_LABEL = { closed: 'full revolution', limited: 'limited', half: 'limited on one side', unbounded: 'no limit' };
  /** Tooltip explaining the range of mode m. */
  function rangeTitle(m) {
    const r = S.sim.ranges && S.sim.ranges[m];
    if (!r) return '';
    const span = `${fmtT(r.min)} to ${fmtT(r.max)}`;
    switch (r.kind) {
      case 'closed': return `Full revolution: one loop of the motion (${span}); Play repeats it. The end is the same pose as the start.`;
      case 'limited': return `Limited motion between two limits (${span}); Play turns round at each end.`;
      case 'half': return `Limited on one side (${span}); the other side stops at a fixed travel of ${FREE_TRAVEL}. Play turns round at each end.`;
      default: return `No limit found within ${fmtT(r.searched)} of travel (a loop longer than that is not detected either): showing a fixed range of +-${FREE_TRAVEL} (${span}). Play turns round at each end.`;
    }
  }
  /** Readout of mode m: the angle in degrees for a mode that turns a body once round, otherwise the parameter t. */
  function valueText(m) {
    const sim = S.sim;
    const ref = sim.traj && sim.angleRefs[m];
    const range = sim.ranges && sim.ranges[m];
    if (ref && range && range.kind === 'closed') {
      const turn = ref.sign * (S.q[3 * ref.body + 2] - ref.theta);
      return `${Math.round(((((turn * 180) / Math.PI) % 360) + 360) % 360) % 360}°`;
    }
    return fmtT(sim.traj ? sim.params[m] : 0);
  }

  const fmtT = (v) => (Math.abs(v) < 5e-3 ? 0 : v).toFixed(2);
  function updateMotionUI() {
    if (!S.analysis) return; // before the first refresh
    const sim = S.sim;
    const k = sim.traj ? sim.traj.count : S.analysis.nullity;
    if (!sim.traj && k > 0 && !sim.rebaseTimer) {
      // Build the trajectory shortly after the last edit (ranges cost a few ms; edits arrive per mouse move).
      sim.rebaseTimer = setTimeout(() => {
        sim.rebaseTimer = 0;
        if (!sim.traj && S.analysis.nullity > 0) {
          ensureTrajectory();
          updateMotionUI();
        }
      }, 150);
    }
    const slider = $('sim-t');
    const i = Math.min(S.opts.mode, Math.max(0, k - 1));
    $('sim-t-box').style.display = k > 0 ? '' : 'none';
    if (k > 0) {
      const range = sim.ranges ? sim.ranges[i] : FREE_RANGE;
      slider.min = String(range.min);
      slider.max = String(range.max);
      slider.value = String(sim.traj ? sim.params[i] : 0);
      slider.disabled = !sim.traj || !!sim.locked[i];
      $('sim-t-val').textContent = sim.ranges && sim.angleRefs[i] && range.kind === 'closed' ? valueText(i) : `t = ${valueText(i)}`;
      $('sim-t-box').title = sim.ranges ? rangeTitle(i) : 'Scrub the simulated motion (same t, same pose)';
      $('sim-t-label').textContent = k > 1 ? `Mode ${i + 1}` : 'Time';
    }
    // Panel: one row per independent mode when there are several.
    const box = $('modes');
    box.style.display = k > 1 ? '' : 'none';
    if (k <= 1) {
      box.innerHTML = '';
      box.dataset.count = '';
      return;
    }
    if (box.dataset.count !== String(k)) {
      box.dataset.count = String(k);
      box.innerHTML = '<p class="hint">Independent modes: lock one to hold it fixed while the others move.</p>' +
        Array.from({ length: k }, (_, m) =>
          `<div class="mode" data-mode="${m}"><div class="mode-head"><b>Mode ${m + 1}</b>` +
          '<label class="chk"><input type="checkbox" class="mode-lock"> lock</label>' +
          '<button class="mode-play">Play</button></div>' +
          '<div class="hint mode-desc"></div>' +
          '<label class="field"><input type="range" class="mode-t" step="0.01"><output class="mode-val"></output></label></div>').join('');
    }
    box.querySelectorAll('.mode').forEach((row, m) => {
      const range = sim.ranges ? sim.ranges[m] : FREE_RANGE;
      const t = row.querySelector('.mode-t');
      t.min = String(range.min);
      t.max = String(range.max);
      t.value = String(sim.traj ? sim.params[m] : 0);
      t.disabled = !sim.traj || !!sim.locked[m];
      row.querySelector('.mode-val').textContent = valueText(m);
      row.querySelector('.mode-lock').checked = !!sim.locked[m];
      const play = row.querySelector('.mode-play');
      play.classList.toggle('on', !!sim.active[m]);
      play.textContent = sim.active[m] ? 'Pause' : 'Play';
      play.disabled = !sim.traj || !!sim.locked[m];
      row.querySelector('.mode-desc').textContent = (modeDescriptions[m] || '') + (sim.ranges ? ` (${KIND_LABEL[range.kind]})` : '');
      row.title = sim.ranges ? rangeTitle(m) : '';
      row.classList.toggle('current', m === i);
    });
  }

  $('sim-t').addEventListener('input', (e) => {
    const k = S.sim.traj ? S.sim.traj.count : 1;
    scrubMode(Math.min(S.opts.mode, Math.max(0, k - 1)), Number(e.target.value));
  });
  $('modes').addEventListener('input', (e) => {
    const row = e.target.closest('.mode');
    if (row && e.target.classList.contains('mode-t')) scrubMode(Number(row.dataset.mode), Number(e.target.value));
  });
  $('modes').addEventListener('change', (e) => {
    const row = e.target.closest('.mode');
    if (!row || !e.target.classList.contains('mode-lock')) return;
    const m = Number(row.dataset.mode);
    S.sim.locked[m] = e.target.checked;
    if (e.target.checked) S.sim.active[m] = false;
    updateMotionUI();
    scheduleHashUpdate();
  });
  $('modes').addEventListener('click', (e) => {
    const row = e.target.closest('.mode');
    if (!row || !e.target.classList.contains('mode-play')) return;
    const m = Number(row.dataset.mode);
    setSim(true);
    ensureTrajectory();
    if (S.sim.active[m]) {
      S.sim.active[m] = false;
      if (!S.sim.active.some(Boolean)) setPlaying(false);
      else updateMotionUI();
    } else {
      S.sim.active[m] = true;
      if (S.sim.playing) updateMotionUI();
      else setPlaying(true);
    }
  });

  $('chk-sim').addEventListener('change', (e) => setSim(e.target.checked));
  $('btn-play').addEventListener('click', () => {
    if (S.sim.playing) return setPlaying(false);
    setSim(true);
    if (!S.analysis.nullity) return message('No motion: the system is rigid.');
    ensureTrajectory();
    if (!S.sim.locked.some((l) => !l)) return message('Every mode is locked: unlock one to animate.');
    S.sim.active = S.sim.locked.map((l) => !l);
    setPlaying(true);
  });
  $('btn-reset').addEventListener('click', () => {
    setPlaying(false);
    S.q = K.zeroState(S.sys);
    dropTrajectory();
    refresh();
  });
  $('btn-clear').addEventListener('click', () => {
    setPlaying(false);
    S.chain = null;
    S.preview = null;
    S.scene = M.createScene();
    S.sel = null;
    S.poleSel.clear();
    edited();
  });
  $('btn-fit').addEventListener('click', fitView);
  $('btn-panel').addEventListener('click', () => {
    document.body.classList.toggle('panel-hidden');
    requestAnimationFrame(resize);
  });

  const exampleSelect = $('sel-example');
  exampleSelect.append(new Option('Load example...', ''));
  for (const [key, ex] of Object.entries(M.EXAMPLES)) exampleSelect.append(new Option(ex.label, key));
  exampleSelect.addEventListener('change', () => {
    if (exampleSelect.value) loadExample(exampleSelect.value);
    exampleSelect.value = '';
  });

  function loadExample(key) {
    const ex = M.EXAMPLES[key] || M.EXAMPLES.mechanism;
    setPlaying(false);
    S.chain = null;
    S.preview = null;
    S.scene = ex.build();
    S.sel = null;
    S.poleSel.clear();
    S.opts.mode = 0;
    edited();
    fitView();
  }

  // ------------------------------------------------------------- share link
  // The whole setup lives in the URL hash (`s=`, see serialize.js), so the address bar is always a shareable link.

  /** The shareable state of the app as a serialize.js setup (scene is copied, not referenced). */
  function captureSetup() {
    return {
      scene: M.cloneScene(S.scene),
      flags: Object.fromEntries(VIEW_FLAGS.map((name) => [name, !!S.opts[name]])),
      scale: S.opts.scale,
      mode: S.opts.mode,
      picks: [...S.poleSel],
      locked: S.sim.locked.map(Boolean),
    };
  }

  /** Replace the app state by a validated setup from serialize.js. */
  function applySetup(setup) {
    setPlaying(false);
    S.chain = null;
    S.preview = null;
    S.scene = M.cloneScene(setup.scene);
    S.sel = null;
    Object.assign(S.opts, setup.flags, { scale: setup.scale, mode: setup.mode });
    S.poleSel = new Set(setup.picks);
    S.sim.locked = setup.locked.slice();
    syncOptInputs();
    edited();
    fitView();
  }

  /** Debounced: write the current setup into the address bar (history.replaceState, no new history entries). */
  function scheduleHashUpdate() {
    if (!share.live) return;
    clearTimeout(share.timer);
    share.timer = setTimeout(writeHash, 400);
  }
  async function writeHash() {
    const seq = ++share.seq;
    const encoded = await encodeSetup(captureSetup());
    if (seq !== share.seq || encoded === share.last) return; // superseded, or nothing changed since the last write
    share.last = encoded;
    try {
      history.replaceState(null, '', '#' + hashWithSetup(parseHash(window.location.hash), encoded));
    } catch (err) {
      share.live = false; // e.g. sandboxed frame: keep working, only the address bar stays untouched
      console.warn('Could not update the address bar:', err);
    }
  }

  /** Absolute URL of the current setup; `embed` makes it the read-only viewer used inside an iframe. */
  async function setupUrl({ embed }) {
    let hash = parseHash(window.location.hash);
    if (embed) {
      for (const k of ['toolbar', 'panel', 'edit', 'view', 'embed']) hash.delete(k);
      hash = new URLSearchParams([['embed', '1'], ...hash]); // embed first: survives truncation / bad escaping of what follows
    }
    return window.location.href.split('#')[0] + '#' + hashWithSetup(hash, await encodeSetup(captureSetup()));
  }

  /** Clipboard write with the legacy fallback; last resort is a prompt the user can copy from. */
  async function copyText(text) {
    try {
      await navigator.clipboard.writeText(text);
      return true;
    } catch {
      /* Clipboard API missing or denied (insecure origin, iframe policy): try the legacy path below. */
    }
    const area = Object.assign(document.createElement('textarea'), { value: text });
    area.style.cssText = 'position:fixed;left:-1000px;top:0;opacity:0';
    document.body.append(area);
    area.select();
    let ok = false;
    try {
      ok = document.execCommand('copy');
    } catch (err) {
      console.warn('Legacy copy failed:', err);
    }
    area.remove();
    if (!ok) window.prompt('Copy this text:', text);
    return ok;
  }

  async function copyShare(embed) {
    const url = await setupUrl({ embed });
    const text = embed
      ? `<iframe src="${url.replace(/&/g, '&amp;')}" style="width:100%;height:480px;border:1px solid #ccc" title="Statics simulator" loading="lazy"></iframe>`
      : url;
    const ok = await copyText(text);
    const long = url.length > 8000 ? ` Note: it is ${url.length} characters long; some services truncate very long links.` : '';
    message((ok ? (embed ? 'Iframe code copied.' : 'Link copied.') : 'Copy it from the dialog.') + long, 4000);
  }
  $('btn-copy-link').addEventListener('click', () => copyShare(false));
  $('btn-copy-iframe').addEventListener('click', () => copyShare(true));

  // ----------------------------------------------------------------- panels

  /** Show the values of S.opts in the display controls. */
  function syncOptInputs() {
    for (const input of document.querySelectorAll('[data-opt]')) {
      const key = input.dataset.opt;
      if (input.type === 'checkbox') input.checked = S.opts[key];
      else input.value = S.opts[key];
    }
    $('scale-val').textContent = S.opts.scale.toFixed(2);
  }
  syncOptInputs();
  for (const input of document.querySelectorAll('[data-opt]')) {
    const key = input.dataset.opt;
    input.addEventListener('input', () => {
      S.opts[key] = input.type === 'checkbox' ? input.checked : Number(input.value);
      if (key === 'scale') $('scale-val').textContent = S.opts.scale.toFixed(2);
      draw();
      scheduleHashUpdate();
    });
  }
  $('opt-mode').addEventListener('change', (e) => {
    S.opts.mode = Number(e.target.value);
    refresh();
  });
  $('btn-sel-all').addEventListener('click', () => {
    S.items.forEach((it) => S.poleSel.add(it.id));
    refresh();
    renderProps();
  });
  $('btn-sel-none').addEventListener('click', () => {
    S.poleSel.clear();
    refresh();
    renderProps();
  });

  const lastText = new Map();
  function setHtml(el, html) {
    if (lastText.get(el) === html) return;
    lastText.set(el, html);
    el.innerHTML = html;
  }
  const fmt = (v) => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(3).replace(/\.?0+$/, '');
  const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]);

  function updatePanels() {
    const a = S.analysis;
    const d = K.describe(a);
    $('status').textContent = d.short;
    const unattached = S.scene.supports.filter((s) => !S.sys.supportInfo.get(s.id).active).map((s) => `${s.type} at (${fmt(s.p.x)}, ${fmt(s.p.y)})`);
    setHtml($('analysis'),
      `<div class="big">${esc(d.short)}</div>` +
      `Bodies n = ${a.n}, constraint equations = ${a.eq}<br>` +
      `Grübler DOF 3n − eq = ${a.grubler}<br>` +
      `Instantaneous mobility = ${a.nullity}, redundant equations = ${a.redundant}<br>` +
      `<span class="hint">${esc(d.long)}</span>` +
      (unattached.length ? `<br><span class="hint">Not attached to a body (ignored): ${unattached.join(', ')}</span>` : ''));

    const modeSel = $('opt-mode');
    if (modeSel.options.length !== Math.max(1, a.nullity)) {
      modeSel.innerHTML = '';
      for (let k = 0; k < Math.max(1, a.nullity); k++) modeSel.append(new Option(`Mode ${k + 1}`, String(k)));
    }
    modeSel.value = String(S.opts.mode);
    modeSel.disabled = a.nullity === 0;

    const itemName = (it) => `${it.label} ${it.kind === 'line' ? 'beam' : 'pole'}`;
    const itemById = new Map(S.items.map((it) => [it.id, it]));
    const itemLabel = (id) => itemById.get(id)?.label ?? `#${id}`;
    const lines = S.items.filter((it) => S.poleSel.has(it.id)).map(itemName);
    let html = viewer ? pickList() : lines.length ? `<p>Picked: ${lines.join(', ')}</p>` : '<p class="hint">Use the "Pick poles" tool or the checkbox on a support.</p>';
    if (S.intersections.length) {
      html += '<ul>' + S.intersections.map((x, k) => {
        const r = x.result;
        const what = r.kind === 'point'
          ? `(${fmt(r.point.x)}, ${fmt(r.point.y)})`
          : r.kind === 'parallel'
            ? `∞, direction (${fmt(r.direction.x)}, ${fmt(r.direction.y)})`
            : 'coincident beams (no single point)';
        return `<li>X${k + 1} = ${itemLabel(x.a)} ∩ ${itemLabel(x.b)}: ${what}</li>`;
      }).join('') + '</ul>';
    }
    const derived = derivedPoles(S.poles);
    if (derived.length) {
      html += '<p>Derived with Kennedy’s theorem:</p><ul>' + derived.map((p) => (p.kind === 'point'
        ? `<li>${poleName(...p.pair)} = (${fmt(p.point.x)}, ${fmt(p.point.y)})</li>`
        : `<li>${poleName(...p.pair, true)}, direction (${fmt(p.direction.x)}, ${fmt(p.direction.y)})</li>`)).join('') + '</ul>';
    }
    setHtml($('poles-list'), html);
  }

  /** Viewer: one checkbox per visible pole beam / pole marker (the editor picks them with the Pick poles tool). */
  function pickList() {
    if (!S.items.length) return '<p class="hint">No pole beams or poles are shown.</p>';
    return '<div class="picks">' + S.items.map((it) =>
      `<label class="chk"><input type="checkbox" data-pick="${it.id}"${S.poleSel.has(it.id) ? ' checked' : ''}> ${esc(it.label)} ${it.kind === 'line' ? 'beam' : 'pole'}</label>`).join('') + '</div>';
  }
  $('poles-list').addEventListener('change', (e) => {
    if (e.target.dataset.pick) togglePole(Number(e.target.dataset.pick));
  });

  // -- property panel (rebuilt on selection / edits, never during playback)

  function mk(tag, props = {}, ...children) {
    const el = Object.assign(document.createElement(tag), props);
    el.append(...children);
    return el;
  }
  function checkbox(label, checked, onChange) {
    const input = mk('input', { type: 'checkbox', checked });
    input.addEventListener('change', () => onChange(input.checked));
    return mk('label', { className: 'chk' }, input, label);
  }
  function numberField(label, value, step, onChange) {
    const input = mk('input', { type: 'number', value: String(+Number(value).toFixed(4)), step: String(step) });
    input.addEventListener('change', () => {
      if (input.value !== '' && Number.isFinite(Number(input.value))) onChange(Number(input.value));
    });
    return mk('label', { className: 'field' }, label, input);
  }

  function renderProps() {
    const box = $('props');
    if (!box) return; // viewer: the property panel is not part of the page
    box.innerHTML = '';
    const item = findSelected();
    if (!item) {
      box.append(mk('p', { className: 'hint' }, 'Nothing selected. Click a body or support with the Select tool.'));
      return;
    }
    if (S.sel.kind === 'body') {
      const p = item.pts;
      const num = S.scene.bodies.indexOf(item) + 1; // body number as drawn on the canvas
      const segs = M.bodySegments(item.kind, p, item.segs);
      const info = segs.length === 1
        ? `Rod ${num}: length ${fmt(G.dist(p[0], p[1]))}, angle ${fmt((Math.atan2(p[1].y - p[0].y, p[1].x - p[0].x) * 180) / Math.PI)}°`
        : `Line body ${num}: ${segs.length} segments, total length ${fmt(segs.reduce((sum, [i, j]) => sum + G.dist(p[i], p[j]), 0))}`;
      box.append(mk('p', {}, info));
      box.append(mk('button', { textContent: 'Delete body', onclick: () => { M.removeBody(S.scene, item.id); S.sel = null; edited(); } }));
      return;
    }
    const s = item;
    const info = S.sys.supportInfo.get(s.id);
    const names = { pin: 'Pin support', roller: 'Roller support', joint: 'Joint' };
    box.append(mk('p', {}, names[s.type] + (info.active ? ` on body ${info.bodies.map((i) => i + 1).join(', ')}` : '')));
    if (!info.active) box.append(mk('p', { className: 'hint' }, 'Not attached to enough bodies - ignored by the analysis.'));
    box.append(numberField('x', s.p.x, 0.5, (v) => { s.p.x = v; edited(); }));
    box.append(numberField('y', s.p.y, 0.5, (v) => { s.p.y = v; edited(); }));
    if (s.type === 'roller') {
      box.append(numberField('Slide direction (°)', s.angleDeg, 5, (v) => { s.angleDeg = v; edited(); }));
      box.append(checkbox('Lock sliding (acts as pin)', s.locked, (v) => { s.locked = v; edited(); }));
    }
    if (s.type === 'pin') box.append(checkbox('Clamped (also blocks rotation)', s.clamped, (v) => { s.clamped = v; edited(); }));
    box.append(checkbox(s.type === 'roller' && !s.locked ? 'Show pole beam' : 'Show pole marker', s.showBeam, (v) => { s.showBeam = v; draw(); scheduleHashUpdate(); }));
    if (S.items.some((it) => it.id === s.id)) {
      box.append(checkbox('Pick for intersections', S.poleSel.has(s.id), () => togglePole(s.id)));
    } else if (s.type === 'pin' && s.clamped) {
      box.append(mk('p', { className: 'hint' }, 'A clamped pin has no pole (the body cannot move).'));
    }
    box.append(mk('button', { textContent: 'Delete support', onclick: () => { M.removeSupport(S.scene, s.id); S.sel = null; edited(); } }));
  }

  // --------------------------------------------------------------- keyboard

  window.addEventListener('keydown', (e) => {
    if (viewer) return;
    if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
    if ((e.key === 'Escape' || e.key === 'Enter') && S.chain) {
      finishChain(); // commits when it has a segment; a second Esc then deselects as usual
    } else if (e.key === 'Escape') {
      S.drag = null;
      S.preview = null;
      setTool('select');
      selectItem(null);
    } else if ((e.key === 'Delete' || e.key === 'Backspace') && S.sel) {
      if (S.sel.kind === 'body') M.removeBody(S.scene, S.sel.id);
      else M.removeSupport(S.scene, S.sel.id);
      S.sel = null;
      edited();
    }
  });

  // ------------------------------------------------------------------- viewer

  /** Strip every editing control from the page and gather the display + simulation controls in the sidebar. */
  function setupViewer() {
    document.body.classList.add('viewer');
    document.body.classList.toggle('view-fixed', !zoomEnabled);
    document.querySelectorAll('[data-edit-only]').forEach((el) => el.remove());
    const motion = document.createElement('section');
    motion.innerHTML = '<h2>Motion</h2><div id="motion-box"></div>';
    $('panel').prepend(motion);
    $('motion-box').append($('sim-t-box'), $('btn-play'), $('btn-reset'));
    $('viewer-bar').append($('btn-fit'), $('btn-panel'));
    $('btn-panel').textContent = 'Controls';
    $('btn-panel').title = 'Show / hide the controls';
    canvas.style.cursor = zoomEnabled ? 'grab' : 'default';
  }

  // ------------------------------------------------------------------- boot

  new ResizeObserver(resize).observe($('stage'));
  setTool('select');
  const embedded = optionFlag(params, 'embed') === true;
  document.body.classList.toggle('embed', embedded);
  document.body.classList.toggle('no-toolbar', optionFlag(params, 'toolbar') === false);
  const panelWanted = optionFlag(params, 'panel') ?? ((viewer || !embedded) && window.innerWidth > 720);
  if (viewer) setupViewer();
  document.body.classList.toggle('panel-hidden', !panelWanted);
  resize();
  if (optionFlag(params, 'displaced') === true) {
    S.opts.displaced = true;
    document.querySelector('[data-opt="displaced"]').checked = true;
  }
  if (initial.setup) applySetup(initial.setup);
  else loadExample(params.get('example') || 'mechanism');
  if (initial.error) message(`Could not load the setup from the link (${initial.error.message}). Showing the default example instead.`, 12000);
  share.live = !viewer; // a viewer never rewrites the address bar
  // Baseline: only changes relative to the loaded state rewrite the address bar.
  encodeSetup(captureSetup()).then((encoded) => { share.last = encoded; });
}
