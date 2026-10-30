// Simulation controls: the Simulate switch, Play / Reset, the time slider, the per-mode rows (value slider, lock, Play,
// driver selector), the dead-point rule and the lazy "computing range..." handling. The motion maths is in sim/.
// Mount once, after the markup exists; all element ids it uses are required.

import * as K from '../../core/kinematics.js';
import { FREE_TRAVEL, PLAY_FRAMES, RANGE_SLICE_MS, STALE_RANGES_DEBOUNCE_MS, TRAJECTORY_DEBOUNCE_MS, KIND_RETRY_MS, NOTICE_LONG_MS } from '../../constants.js';
import { reportError, reportWarning } from '../errors.js';
import { createTrajectory, describeMode } from '../../sim/trajectory.js';
import { advanceParams, paramFromValue, valueFromParam, displayRange, startingArc } from '../../sim/playback.js';
import { byId } from '../dom.js';
import { fmtT } from '../format.js';

/**
 * Mounts the simulation controls and returns the entry points the rest of the app needs.
 *  - state: the app state; `state.sim` holds the trajectory, ranges, values and playback flags (see state.js).
 *  - message(text, ms?): transient notice.
 *  - refresh(): re-derive and redraw after the pose changed.
 *  - scheduleHashUpdate(): driver selection and locks are part of shared links.
 *  - instrument(name, fn): wraps a hot function for frame timing (?perf=1); identity when not profiling.
 * Returns { setPlaying(on), dropTrajectory(), update() }: stop/start playback, forget the trajectory because the pose was
 * changed by something else, and refresh every control from state (call after each model refresh).
 */
export function mountSimPanel({ state, message, refresh, scheduleHashUpdate, instrument = (name, fn) => fn }) {
  const S = state;

  function setSim(on) {
    S.sim.on = on;
    byId('chk-sim').checked = on;
    if (!on) setPlaying(false);
  }
  function setPlaying(on) {
    const wasPlaying = S.sim.playing;
    S.sim.playing = on;
    if (!on) S.sim.active = S.sim.active.map(() => false);
    if (wasPlaying && !on) markRangesStale();
    byId('btn-play').textContent = on ? 'Pause' : 'Play';
    byId('btn-play').classList.toggle('on', on);
    if (on) requestAnimationFrame(tick);
    updateMotionUI();
  }

  // -- finite motion along the independent modes (maths in sim/)

  let modeDescriptions = [];
  /** "Mode N: <driver>": the slot number plus what drives it (the slot keeps both when its driver is replaced). */
  const modeTitle = (m) => (modeDescriptions[m] ? `Mode ${m + 1}: ${modeDescriptions[m]}` : `Mode ${m + 1}`);

  /** The pose was changed by something other than the trajectory: rebase it at the current pose when next needed. */
  function dropTrajectory() {
    clearTimeout(S.sim.staleTimer);
    S.sim.staleTimer = 0;
    S.sim.stale = false;
    clearTimeout(S.sim.rangeTimer);
    S.sim.rangeTimer = 0;
    S.sim.prior = null;
    S.sim.traj = null;
    S.sim.ranges = null;
    S.sim.angleMemory.clear();
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
   * Builds the trajectory at the current pose. Every mode is classified there (the other modes held at this pose), but
   * lazily: the range of a mode is installed by installRange() when its classification finishes (scheduleRanges), the
   * displayed mode first; until then sim.ranges[i] is null (the slider says "computing range"). A carried mode keeps its
   * previous display range meanwhile. `keep` = { values, ranges } carries over the displayed values and the fixed free-side
   * travel when an existing trajectory is rebased at its current pose.
   */
  function rebuildTrajectory(keep) {
    const sim = S.sim;
    clearTimeout(sim.rebaseTimer);
    clearTimeout(sim.staleTimer);
    sim.rebaseTimer = 0;
    sim.staleTimer = 0;
    sim.stale = false;
    clearTimeout(sim.rangeTimer);
    sim.rangeTimer = 0;
    let traj;
    try {
      traj = createTrajectory(S.sys, S.q, { drivers: sim.selection || undefined });
      if (traj.warnings.length) {
        // The stored selection does not fit this mechanism / pose: the default one is used and the choice is forgotten.
        for (const warning of traj.warnings) reportWarning('Motion', warning);
        message(`Motion: ${traj.warnings.join('; ')}`, NOTICE_LONG_MS);
        if (sim.selection) {
          sim.selection = null;
          scheduleHashUpdate();
        }
      }
    } catch (err) {
      // No usable modes for this mechanism: say so and carry on without a trajectory instead of leaving it half built.
      reportError('Motion unavailable', err);
      const pose = S.q.slice();
      traj = { count: 0, modes: [], drivers: [], candidates: [], warnings: [], poseAt: () => ({ q: pose.slice(), params: [], blocked: false }), modeRange: () => null };
    }
    const k = traj.count;
    const carried = !!keep && keep.values.length === k;
    const prior = carried ? { values: keep.values.slice(), ranges: keep.ranges.slice(), angleRefs: sim.angleRefs.slice() } : null;
    sim.traj = traj;
    sim.prior = prior;
    sim.base = S.q.slice();
    sim.rangeDone = new Array(k).fill(false);
    sim.wanted = -1;
    // Until installRange() runs a mode shows its previous range (carried) or none (null); its offset is the driver value.
    sim.offsets = Array.from({ length: k }, (_, i) => (prior && prior.ranges[i] ? prior.values[i] : traj.drivers[i] ? traj.drivers[i].value(S.q) : 0));
    sim.angleRefs = prior ? prior.angleRefs.slice() : new Array(k).fill(null);
    sim.ranges = Array.from({ length: k }, (_, i) => (prior && prior.ranges[i]) || null);
    sim.params = sim.offsets.slice();
    if (!carried) {
      sim.dir = new Array(k).fill(1);
      sim.active = new Array(k).fill(false);
      sim.arc = new Array(k).fill(0);
      sim.deadCount = new Array(k).fill(0);
    }
    if (sim.locked.length !== k) sim.locked = new Array(k).fill(false);
    S.opts.mode = Math.min(S.opts.mode, Math.max(0, k - 1));
    // Several modes are named after their driver coordinate; a single mode after the body that moves most.
    modeDescriptions = traj.modes.map((m, i) => (traj.drivers[i] ? traj.drivers[i].label : describeMode(S.sys, S.q, m)));
    if (k > 0) scheduleRanges(traj);
  }
  /**
   * Installs the finished classification `info` of mode i (see rebuildTrajectory): angle reference, offset, display range
   * and, for a mode that had no range before, its starting arc.
   */
  function installRange(i, info) {
    const sim = S.sim;
    const k = sim.traj.count;
    const prior = sim.prior;
    const known = !!prior && !!prior.ranges[i];
    const before = known && prior.angleRefs[i];
    const base = sim.base;
    let ref = null;
    if (info.angle) {
      if (before && before.body === info.angle.body && before.sign === info.angle.sign) {
        sim.angleMemory.set(before.body, before.theta);
        ref = before;
      } else {
        const remembered = sim.angleMemory.get(info.angle.body);
        const theta = remembered !== undefined ? remembered : base[3 * info.angle.body + 2];
        sim.angleMemory.set(info.angle.body, theta);
        ref = { body: info.angle.body, sign: info.angle.sign, theta };
      }
    }
    let offset = info.value;
    if (info.kind === 'closed') {
      if (known) offset = prior.values[i];
      else if (!ref) offset = 0;
      else {
        const turn = ref.sign * (base[3 * ref.body + 2] - ref.theta);
        offset = ((turn % info.period) + info.period) % info.period;
      }
    }
    sim.angleRefs[i] = ref;
    sim.offsets[i] = offset;
    // The arcs of a mode (dead-point playback, see playback.js) are followed only when it is the single mode.
    const range = displayRange(info, offset, known ? prior.ranges[i] : undefined);
    sim.ranges[i] = k > 1 ? { ...range, arcs: undefined } : range;
    if (!known) {
      sim.params[i] = offset;
      sim.arc[i] = range.branch;
      sim.dir[i] = 1;
      sim.deadCount[i] = 0;
      if (k === 1) applyStartRule();
    }
  }
  /** Mode m was asked for (hover, focus, Play): its range is computed before the others. */
  function wantRange(m) {
    const sim = S.sim;
    if (sim.traj && m >= 0 && m < sim.rangeDone.length && !sim.rangeDone[m]) sim.wanted = m;
  }
  /**
   * Computes the missing ranges one slice (RANGE_SLICE_MS) per timer tick so that no task blocks the page: the requested
   * mode first, then the displayed one, then the rest in order. Stops when the trajectory has been replaced.
   */
  function scheduleRanges(traj) {
    const sim = S.sim;
    const pump = () => {
      sim.rangeTimer = 0;
      if (sim.traj !== traj) return;
      const order = [sim.wanted, S.opts.mode, ...traj.modes.map((_, m) => m)];
      const next = order.find((m) => m >= 0 && m < traj.count && !sim.rangeDone[m]);
      if (next === undefined) return;
      const step = traj.rangeStep(next, RANGE_SLICE_MS);
      if (step.done) {
        sim.rangeDone[next] = true;
        if (sim.wanted === next) sim.wanted = -1;
        installRange(next, step.range);
        updateMotionUI();
      }
      sim.rangeTimer = setTimeout(pump, 0);
    };
    sim.rangeTimer = setTimeout(pump, 0);
  }
  /**
   * The dead-point rule for a single limited mode that sits at its base pose: when that pose is itself a dead point the
   * rule picks the arc and direction it starts on (counted as the first dead point of the mode).
   */
  function applyStartRule() {
    const sim = S.sim;
    const range = sim.ranges && sim.ranges[0];
    if (!range || sim.traj.count !== 1 || Math.abs(sim.params[0] - sim.offsets[0]) > 1e-9) return;
    const start = startingArc(range, sim.deadPoint, 0);
    sim.arc[0] = start.arc;
    sim.dir[0] = start.dir;
    sim.deadCount[0] = start.dead ? 1 : 0;
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
    }, STALE_RANGES_DEBOUNCE_MS);
  }

  /** Trajectory parameters t for the display values u (per-mode offsets / wrap-around of closed modes). */
  const paramsOf = (u) => u.map((v, i) => (S.sim.ranges[i] ? paramFromValue(S.sim.ranges[i], S.sim.offsets[i], v) : 0));

  /** Moves to the pose for display values u (the pose is a pure function of u); returns false when blocked. */
  function applyValues(u) {
    const sim = S.sim;
    const pose = sim.traj.poseAt(paramsOf(u), { arcs: sim.arc });
    S.q = pose.q;
    sim.params = pose.blocked ? pose.params.map((t, i) => (sim.ranges[i] ? valueFromParam(sim.ranges[i], sim.offsets[i], t) : sim.params[i])) : u.slice();
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
    // A mode whose range is still being computed does not move yet; playback waits for it.
    const playing = sim.active.map((a, i) => a && !!sim.ranges[i]);
    if (!playing.some((a, i) => a && !sim.locked[i])) {
      sim.active.forEach((a, i) => a && wantRange(i));
      requestAnimationFrame(tick);
      return;
    }
    const speeds = sim.ranges.map((r) => (r ? Math.max(1e-4, (r.max - r.min) / PLAY_FRAMES) : 0));
    const next = advanceParams({ values: sim.params, dir: sim.dir, playing, locked: sim.locked, ranges: sim.ranges, arc: sim.arc, deadCount: sim.deadCount, deadPoint: sim.deadPoint }, speeds);
    const pose = traj.poseAt(paramsOf(next.values), { arcs: next.arc });
    if (pose.blocked) {
      // The combined motion hit a limit that no single mode reaches: turn the playing modes round.
      sim.dir = sim.dir.map((d, i) => (sim.active[i] && !sim.locked[i] ? -d : d));
    } else {
      S.q = pose.q;
      sim.params = next.values;
      sim.dir = next.dir;
      sim.arc = next.arc;
      sim.deadCount = next.deadCount;
    }
    refresh();
    if (sim.playing) requestAnimationFrame(tick);
  }

  /** Sets one mode value (slider scrub): pauses playback and shows the pose for the new values. */
  function scrubMode(i, value) {
    setSim(true);
    setPlaying(false);
    ensureTrajectory();
    if (!S.sim.ranges || !S.sim.ranges[i]) {
      wantRange(i);
      return;
    }
    const u = S.sim.params.slice();
    u[i] = value;
    if (!applyValues(u)) message('Motion blocked: that combination of modes is out of reach.');
    S.opts.mode = i;
    markRangesStale();
    refresh();
  }

  /**
   * Another driver for mode m (or for the single mode): the trajectory is rebased at the current pose with the new
   * selection, so the pose does not jump. Locks and the playing state are kept; arcs and directions restart.
   */
  function changeDriver(m, id) {
    const sim = S.sim;
    const traj = ensureTrajectory();
    const ids = traj.count > 1 ? traj.drivers.map((d) => d.id) : [id];
    if (traj.count > 1) ids[m] = id;
    const active = sim.active.slice();
    sim.selection = ids;
    rebuildTrajectory(null);
    if (active.length === sim.traj.count) sim.active = active;
    scheduleHashUpdate();
    refresh();
  }

  const DRIVER_KIND_TEXT = { full: 'full rotation', partial: 'partial', unbounded: 'unbounded' };
  const driverMenus = new WeakMap(); // trajectory -> per-mode menu { current, sig, options }
  /** Options of the driver selector of mode m: the valid candidates (the current driver always included), built once per trajectory. */
  function driverMenu(m) {
    const sim = S.sim;
    const traj = sim.traj;
    const range = sim.ranges && sim.ranges[m];
    if (!traj || !range || !range.driver) return null;
    let menus = driverMenus.get(traj);
    if (!menus) driverMenus.set(traj, (menus = []));
    if (!menus[m]) {
      const kinds = (kindsOf.get(traj) || new Map()).get(m) || {}; // the annotations arrive later (scheduleKinds)
      const entries = (traj.candidates[m] || []).slice();
      if (!entries.some((e) => e.id === range.driver.id)) entries.unshift(range.driver);
      const options = entries.map((e) => ({ id: e.id, text: kinds[e.id] ? `${e.label} - ${DRIVER_KIND_TEXT[kinds[e.id]]}` : e.label }));
      menus[m] = { current: range.driver.id, options, sig: JSON.stringify(options) };
    }
    if (!kindsOf.get(traj) || kindsOf.get(traj).size < traj.count) scheduleKinds(traj);
    return menus[m];
  }
  const kindsOf = new WeakMap(); // trajectory -> Map(mode -> candidateKinds(mode)), filled one mode per timer tick
  const kindsScheduled = new WeakSet();
  /**
   * Classifying every candidate driver of every mode is the most expensive part of a trajectory and only annotates the
   * driver selector, so it runs after the first paint, one mode (the shown one first) per timer tick; the selector shows
   * the plain driver labels until then. Stops when the trajectory has been replaced.
   */
  function scheduleKinds(traj) {
    if (kindsScheduled.has(traj)) return;
    kindsScheduled.add(traj);
    const done = new Map();
    kindsOf.set(traj, done);
    const order = () => {
      const first = Math.min(S.opts.mode, traj.count - 1);
      return [first, ...traj.modes.map((_, m) => m).filter((m) => m !== first)];
    };
    const step = () => {
      if (S.sim.traj !== traj) return;
      const next = order().find((m) => !done.has(m));
      if (next === undefined) return;
      // The ranges the sliders wait for come first.
      if (S.sim.rangeDone.some((d) => !d)) return void setTimeout(step, KIND_RETRY_MS);
      const slice = traj.kindsStep(next, RANGE_SLICE_MS);
      if (slice.done) {
        done.set(next, slice.kinds);
        const menus = driverMenus.get(traj);
        if (menus) menus[next] = undefined;
        updateMotionUI();
      }
      setTimeout(step, 0);
    };
    setTimeout(step, 0);
  }
  /** Shows a menu in a <select>; the options are only replaced when they changed (an open list must not flicker while playing). */
  function fillDriverSelect(select, menu) {
    select.disabled = !menu;
    if (!menu) return;
    if (select.dataset.sig !== menu.sig) {
      select.replaceChildren(...menu.options.map((o) => new Option(o.text, o.id)));
      select.dataset.sig = menu.sig;
    }
    select.value = menu.current;
  }

  const KIND_LABEL = { closed: 'full revolution', limited: 'limited', half: 'limited on one side', unbounded: 'no limit' };
  const isAngle = (range) => !!range && !!range.driver && (range.driver.kind === 'rotation' || range.driver.kind === 'relAngle');
  /** A driver value for display: degrees for rotations, plain number (length units) otherwise. */
  const fmtV = (range, v) => (isAngle(range) ? `${((v * 180) / Math.PI).toFixed(1)}°` : fmtT(v));
  /** Tooltip explaining the range of mode m. */
  function rangeTitle(m) {
    const r = S.sim.ranges && S.sim.ranges[m];
    if (!r) return S.sim.traj ? 'Computing the range of this mode...' : '';
    const driver = r.driver ? r.driver.label : 'the mode';
    const span = `${fmtV(r, r.min)} to ${fmtV(r, r.max)}`;
    const free = `the free side stops at a fixed travel of ${FREE_TRAVEL}${isAngle(r) ? ' rad' : ''}`;
    const unexplored = r.status === 'budget' ? ` Range not fully explored: no limit was found within ${fmtT(r.searched)} of travel.` : '';
    switch (r.kind) {
      case 'closed': return `Full revolution of ${driver}: one loop of the motion (${span}); Play repeats it. The end is the same pose as the start.`;
      case 'limited': return `${driver} runs between two limits (${span}) on the branch of the motion the mechanism is on; at each end Play follows the "At dead points" rule.`;
      case 'half': return `${driver}: limited on one side (${span}); ${free}. Play turns round at each end.${unexplored}`;
      default: return r.status === 'budget'
        ? `Range not fully explored: no limit found within ${fmtT(r.searched)} of travel (a loop longer than that is not detected either). Showing a fixed range of +-${FREE_TRAVEL} (${span}); Play turns round at each end.`
        : `${driver}: no limit on either side (checked ${fmtT(r.searched)} of travel); showing a fixed range of +-${FREE_TRAVEL} (${span}). Play turns round at each end.`;
    }
  }
  /** Readout of mode m: the angle in degrees for a mode that turns a body once round, otherwise the driver value. */
  function valueText(m) {
    const sim = S.sim;
    const ref = sim.traj && sim.angleRefs[m];
    const range = sim.ranges && sim.ranges[m];
    if (ref && range && range.kind === 'closed') {
      const turn = ref.sign * (S.q[3 * ref.body + 2] - ref.theta);
      return `${Math.round(((((turn * 180) / Math.PI) % 360) + 360) % 360) % 360}°`;
    }
    return range && sim.traj ? fmtV(range, sim.params[m]) : '...';
  }

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
      }, TRAJECTORY_DEBOUNCE_MS);
    }
    const slider = byId('sim-t');
    const i = Math.min(S.opts.mode, Math.max(0, k - 1));
    byId('sim-t-box').style.display = k > 0 ? '' : 'none';
    const singleMenu = k === 1 ? driverMenu(0) : null;
    byId('driver-box').style.display = singleMenu ? '' : 'none';
    if (singleMenu) fillDriverSelect(byId('sel-driver'), singleMenu);
    byId('dead-box').style.display = sim.ranges && sim.ranges.some((r) => r && (r.kind === 'limited' || r.kind === 'half')) ? '' : 'none';
    if (k > 0) {
      const range = sim.ranges ? sim.ranges[i] : null; // null: no trajectory yet or the range is being computed
      slider.step = 'any';
      slider.min = range ? String(range.min) : '0';
      slider.max = range ? String(range.max) : '1';
      slider.value = range && sim.traj ? String(sim.params[i]) : '0';
      slider.disabled = !range || !sim.traj || !!sim.locked[i];
      byId('sim-t-val').textContent = !range ? 'computing range...' : sim.angleRefs[i] && range.kind === 'closed' ? valueText(i) : `v = ${valueText(i)}`;
      byId('sim-t-box').title = range ? rangeTitle(i) : 'Computing the range of this mode...';
      byId('sim-t-label').textContent = k > 1 ? modeTitle(i) : 'Time';
    }
    // Panel: one row per independent mode when there are several.
    const box = byId('modes');
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
          `<div class="mode" data-mode="${m}"><div class="mode-head"><b class="mode-title">${modeTitle(m)}</b>` +
          '<label class="chk"><input type="checkbox" class="mode-lock"> lock</label>' +
          '<button class="mode-play">Play</button></div>' +
          '<div class="hint mode-desc"></div>' +
          '<label class="field mode-driver-box" title="What drives this mode. Drivers that would depend on the other modes are not listed."><span>Driven by</span><select class="mode-driver"></select></label>' +
          '<label class="field"><input type="range" class="mode-t" step="0.01"><output class="mode-val"></output></label></div>').join('');
    }
    box.querySelectorAll('.mode').forEach((row, m) => {
      const range = sim.ranges ? sim.ranges[m] : null;
      const t = row.querySelector('.mode-t');
      t.step = 'any';
      t.min = range ? String(range.min) : '0';
      t.max = range ? String(range.max) : '1';
      t.value = range && sim.traj ? String(sim.params[m]) : '0';
      t.disabled = !range || !sim.traj || !!sim.locked[m];
      row.querySelector('.mode-val').textContent = range ? valueText(m) : 'computing range...';
      row.classList.toggle('computing', !range);
      row.querySelector('.mode-lock').checked = !!sim.locked[m];
      const play = row.querySelector('.mode-play');
      play.classList.toggle('on', !!sim.active[m]);
      play.textContent = sim.active[m] ? 'Pause' : 'Play';
      play.disabled = !range || !sim.traj || !!sim.locked[m];
      row.querySelector('.mode-title').textContent = modeTitle(m);
      row.querySelector('.mode-desc').textContent = !range ? (sim.traj ? '(computing range...)' : '') : sim.ranges ? `(${KIND_LABEL[range.kind]}${range.status === 'budget' && range.kind !== 'limited' && range.kind !== 'closed' ? ', range not fully explored' : ''})` : '';
      fillDriverSelect(row.querySelector('.mode-driver'), driverMenu(m));
      row.title = sim.ranges ? rangeTitle(m) : '';
      row.classList.toggle('current', m === i);
    });
  }

  byId('sim-t').addEventListener('input', (e) => {
    const k = S.sim.traj ? S.sim.traj.count : 1;
    scrubMode(Math.min(S.opts.mode, Math.max(0, k - 1)), Number(e.target.value));
  });
  for (const type of ['pointerover', 'focusin']) {
    byId('modes').addEventListener(type, (e) => {
      const row = e.target.closest && e.target.closest('.mode');
      if (row) wantRange(Number(row.dataset.mode));
    });
  }
  byId('modes').addEventListener('input', (e) => {
    const row = e.target.closest('.mode');
    if (row && e.target.classList.contains('mode-t')) scrubMode(Number(row.dataset.mode), Number(e.target.value));
  });
  byId('modes').addEventListener('change', (e) => {
    const row = e.target.closest('.mode');
    if (row && e.target.classList.contains('mode-driver')) return changeDriver(Number(row.dataset.mode), e.target.value);
    if (!row || !e.target.classList.contains('mode-lock')) return;
    const m = Number(row.dataset.mode);
    S.sim.locked[m] = e.target.checked;
    if (e.target.checked) S.sim.active[m] = false;
    updateMotionUI();
    scheduleHashUpdate();
  });
  byId('modes').addEventListener('click', (e) => {
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

  byId('sel-driver').addEventListener('change', (e) => changeDriver(0, e.target.value));
  byId('sel-dead').addEventListener('change', (e) => {
    S.sim.deadPoint = e.target.value;
    if (S.sim.traj && !S.sim.playing) applyStartRule();
  });
  byId('chk-sim').addEventListener('change', (e) => setSim(e.target.checked));
  byId('btn-play').addEventListener('click', () => {
    if (S.sim.playing) return setPlaying(false);
    setSim(true);
    if (!S.analysis.nullity) return message('No motion: the system is rigid.');
    ensureTrajectory();
    if (!S.sim.locked.some((l) => !l)) return message('Every mode is locked: unlock one to animate.');
    S.sim.active = S.sim.locked.map((l) => !l);
    setPlaying(true);
  });
  byId('btn-reset').addEventListener('click', () => {
    setPlaying(false);
    S.q = K.zeroState(S.sys);
    dropTrajectory();
    refresh();
  });

  // Function declarations are mutable bindings: every later call (including requestAnimationFrame(tick)) is timed.
  tick = instrument('tick', tick);
  rebuildTrajectory = instrument('rebuildTrajectory', rebuildTrajectory);
  updateMotionUI = instrument('updateMotionUI', updateMotionUI);

  return { setPlaying, dropTrajectory, update: () => updateMotionUI() };
}
