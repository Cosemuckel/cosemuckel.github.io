// App orchestration: model refresh, drawing, panels, viewer setup and boot. Pointer tools live in tools.js; maths in core/.

import * as M from '../core/model.js';
import * as K from '../core/kinematics.js';
import { derivePoles } from '../core/kennedy.js';
import { render } from '../render/index.js';
import { NOTICE_LONG_MS } from '../constants.js';
import { createPerfLog } from './perf.js';
import { mountSimPanel } from './panels/sim-panel.js';
import { setStatusSink } from './errors.js';
import { listDrivers } from '../sim/drivers.js';
import { slotsOf } from './slots.js';
import { isViewer, optionFlag } from '../io/options.js';
import { fitViewFor, linkView, keepFramedRadius, measureCanvas, sizeBackingStore } from './view.js';
import { createShare } from './share.js';
import { mountAnalysis } from './panels/analysis.js';
import { mountOptions } from './panels/options.js';
import { mountProperties } from './panels/properties.js';
import { mountTools } from './tools.js';
import { createStore, createAppState, findSelected } from './state.js';
import { byId, createNotice } from './dom.js';

export function startApp(params, initial = {}) {
  const canvas = byId('view');
  const ctx = canvas.getContext('2d');
  // Viewer: read-only presentation for iframes (embed=1 or view=1); edit=1 keeps the full editor inside an iframe.
  const viewer = isViewer(params);
  const zoomEnabled = optionFlag(params, 'zoom') !== false; // viewer only: zoom=0 freezes the view (no wheel zoom, no pan)
  const perf = optionFlag(params, 'perf') === true ? createPerfLog() : null; // dev-only frame timing (?perf=1), see perf.js
  const renderFrame = perf ? perf.timed('draw', render) : render;

  const store = createStore(createAppState());
  const S = store.state;

  // ------------------------------------------------------------ model refresh

  /** Scene geometry/supports changed: drop any displacement and rebuild everything. */
  function edited() {
    S.sys = K.buildSystem(S.scene);
    S.q = K.zeroState(S.sys);
    S.sim.selection = null; // driver ids belong to the old bodies / supports
    dropTrajectory();
    if (S.sel && !findSelected(S)) S.sel = null;
    refresh();
    renderProps();
  }

  /** Pose changed (or mode/selection changed): redo analysis-derived data and redraw. */
  function refresh() {
    S.analysis = K.analyze(S.sys, S.q);
    S.opts.mode = Math.min(S.opts.mode, Math.max(0, S.analysis.nullity - 1));
    const slots = slotsOf(S);
    S.modeLabels = slots ? slots.labels : [];
    S.dq = S.analysis.nullity ? K.normalizeMode(S.sys, S.q, slots && slots.motions ? slots.motions[S.opts.mode] : S.analysis.modes[S.opts.mode]) : null;
    S.items = K.poleItems(S.sys, S.q, S.scene);
    const ids = new Set(S.items.map((it) => it.id));
    for (const id of [...S.poleSel]) if (!ids.has(id)) S.poleSel.delete(id);
    S.intersections = K.selectedIntersections(S.items, S.poleSel);
    S.poles = polesFor(S.items, S.sys.n);
    updatePanels();
    updateMotionUI();
    draw();
    scheduleHashUpdate();
  }

  /**
   * Derived poles of the pole items. Kennedy's theorem is O(n^3) per sweep and refresh() also runs when only the mode,
   * a selection or a flag changed, so the previous result is reused while the items (positions, beams, labels) are identical.
   */
  let poleMemo = null;
  function polesFor(items, bodyCount) {
    const same = (a, b) => a.id === b.id && a.kind === b.kind && a.label === b.label && a.point.x === b.point.x && a.point.y === b.point.y
      && (a.dir ? !!b.dir && a.dir.x === b.dir.x && a.dir.y === b.dir.y : !b.dir);
    if (poleMemo && poleMemo.bodyCount === bodyCount && poleMemo.items.length === items.length && items.every((it, k) => same(it, poleMemo.items[k]))) return poleMemo.poles;
    poleMemo = { bodyCount, items, poles: derivePoles(items, bodyCount) };
    return poleMemo.poles;
  }

  // ----------------------------------------------------------------- drawing

  let drawQueued = false;
  function draw() {
    if (drawQueued) return;
    drawQueued = true;
    requestAnimationFrame(() => {
      drawQueued = false;
      renderFrame(ctx, {
        ...S.size, view: S.view, scene: S.scene, sys: S.sys, q: S.q, dq: S.dq,
        items: S.items, intersections: S.intersections, poles: S.poles, opts: S.opts,
        sel: S.sel, poleSel: S.poleSel, preview: S.preview,
      });
    });
  }

  function resize() {
    const previous = S.size;
    S.size = measureCanvas(canvas);
    if (S.viewFromLink) S.view = keepFramedRadius(S.view, previous, S.size); // same framed circle in a resized iframe
    sizeBackingStore(canvas, S.size);
    draw();
  }
  function fitView() {
    S.view = fitViewFor(S.scene, S.size);
    S.viewCustom = false;    S.viewFromLink = false;
    scheduleViewUpdate();
    draw();
  }

  /** Initial framing from a `cam=cx,cy,r` option (or the legacy `view=cx,cy,r`), on top of the fitted view; an invalid value is ignored (fit stays). */
  function applyLinkView() {
    const view = linkView(params, S.size);
    if (!view) return;
    S.view = view;
    S.viewCustom = true;
    S.viewFromLink = viewer;
    draw();
  }

  /** The view moved by hand (wheel / pan): it now belongs in shared links. */
  function viewMoved() {
    S.viewCustom = true;
    scheduleViewUpdate();
  }

  // ------------------------------------------------------------------ messages

  const message = createNotice(byId('msg'));
  setStatusSink(message);
  const share = createShare({ state: S, viewer, message });
  const { scheduleHashUpdate, scheduleViewUpdate } = share;

  function togglePole(id) {
    if (S.poleSel.has(id)) S.poleSel.delete(id);
    else S.poleSel.add(id);
    refresh();
    renderProps();
  }

  const instrument = (name, fn) => (perf ? perf.timed(name, fn) : fn);
  const simPanel = mountSimPanel({ state: S, message, refresh: () => refresh(), scheduleHashUpdate: () => scheduleHashUpdate(), instrument });
  const { setPlaying, dropTrajectory } = simPanel;
  const updateMotionUI = () => simPanel.update();
  byId('btn-clear').addEventListener('click', () => {
    setPlaying(false);
    S.chain = null;
    S.preview = null;
    S.scene = M.createScene();
    S.sel = null;
    S.poleSel.clear();
    edited();
  });
  byId('btn-fit').addEventListener('click', fitView);
  byId('btn-panel').addEventListener('click', () => {
    document.body.classList.toggle('panel-hidden');
    requestAnimationFrame(resize);
  });

  const exampleSelect = byId('sel-example');
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
  // Address-bar sync and Copy link live in share.js; applying a loaded setup needs the app functions, so it stays here.

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
    optionsPanel.sync();
    edited();
    // Ids of drivers that do not exist cannot be written back into a link: ignore the whole selection with a message. Whether
    // the drivers are independent at the pose is checked when the trajectory is built.
    const known = new Set(listDrivers(S.sys).map((d) => d.id));
    const unknown = (setup.drivers || []).find((id) => !known.has(id));
    S.sim.selection = setup.drivers && unknown === undefined ? setup.drivers.slice() : null;
    if (unknown !== undefined) message(`Motion: driver selection ignored: unknown driver '${unknown}'`, NOTICE_LONG_MS);
    fitView();
  }

  byId('btn-copy-link').addEventListener('click', () => share.copyShare(false));
  byId('btn-copy-iframe').addEventListener('click', () => share.copyShare(true));

  // ----------------------------------------------------------------- panels

  const optionsPanel = mountOptions({ state: S, draw, scheduleHashUpdate, refresh: () => refresh() });
  const analysisPanel = mountAnalysis({ state: S, viewer, togglePole, onPicksChanged: () => { refresh(); renderProps(); } });
  function updatePanels() {
    analysisPanel.update(S);
  }

  const properties = mountProperties({ state: S, edited, draw, scheduleHashUpdate, togglePole });
  const renderProps = () => properties.render();

  const { setTool } = mountTools({
    state: S, store, canvas, viewer, zoomEnabled, message, edited, draw, renderProps, togglePole, dropTrajectory, viewMoved,
    refresh: () => refresh(),
  });

  // ------------------------------------------------------------------- viewer

  /** Strip every editing control from the page and gather the display + simulation controls in the sidebar. */
  function setupViewer() {
    document.body.classList.add('viewer');
    document.body.classList.toggle('view-fixed', !zoomEnabled);
    document.querySelectorAll('[data-edit-only]').forEach((el) => el.remove());
    const motion = document.createElement('section');
    motion.innerHTML = '<h2>Motion</h2><div id="motion-box"></div>';
    byId('panel').prepend(motion);
    byId('motion-box').append(byId('sim-t-box'), byId('driver-box'), byId('dead-box'), byId('btn-play'), byId('btn-reset'));
    byId('viewer-bar').append(byId('btn-fit'), byId('btn-panel'));
    byId('btn-panel').textContent = 'Controls';
    byId('btn-panel').title = 'Show / hide the controls';
    canvas.style.cursor = zoomEnabled ? 'grab' : 'default';
  }

  // ------------------------------------------------------------------- boot

  if (perf) {
    // Function declarations are mutable bindings: every later call (including requestAnimationFrame(tick)) is timed.
    refresh = perf.timed('refresh', refresh);
    updatePanels = perf.timed('updatePanels', updatePanels);
  }
  new ResizeObserver(resize).observe(byId('stage'));
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
  applyLinkView();
  if (initial.error) share.reportLoadError(initial.error);
  share.start(); // baseline: only changes relative to the loaded state rewrite the address bar
}
