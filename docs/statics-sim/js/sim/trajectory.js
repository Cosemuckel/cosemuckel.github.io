// @ts-check
// Deterministic finite motion along the mechanism's independent degrees of freedom (pure maths, no DOM).
//
// A mechanism with k = nullity(J) degrees of freedom is steered by k numbers t = (t1..tk), one per mode. Mode i has a
// DRIVER g_i(q) (see drivers.js); t_i is the change of that driver from its value at the base pose, in driver units
// (radians for a rotation / relative angle, length for a translation / slide). The user-facing value of a mode is
// therefore the driver value v = g_i(base) + t_i.
//
// k = 1: the driver is the one given in options.drivers, otherwise the first driver of the canonical order that moves
// along the motion (drivers.js; no curve walk is involved). The curve is the null-space curve through the base pose.
//
// k > 1: k drivers are selected (options.drivers, a list in slot order, or by default the canonical-first independent
// ones, see drivers.js), such that holding any k - 1 of them leaves exactly one degree of freedom. Slot (mode) i =
// vary driver i while every other driver is held at its current value: a true 1-DOF curve. The selection, the slot
// order and the candidate lists are a function of the mechanism, not of the pose the trajectory is built at.
//
// The pose for t_i is the point of the mode's curve at which the driver has changed by t_i, on the monotone ARC of
// the driver that contains the pose the mode starts from (curve.solveDriver): the walk goes outward from that pose
// in the sense that moves the driver towards the target and stops at an extremum of the driver (a stroke limit of a
// slider, a turning point of a rocker) or at a limit of the curve. A sliding driver does not say in which sense the
// rest of the mechanism moves; this rule keeps the motion on the side of the full solution where it started.
// The exception is a driver that winds round (a crank): its pose is periodic with the winding, kind 'closed'.
// For several modes the pose is the composition of the mode solves in mode order (mode 1 from the base pose, then
// mode 2 from the pose reached, holding the then-current values of the other drivers, and so on): a pure function
// of (system, base pose, selection, t), whatever the user did before. Changing t_j never changes the driver
// coordinates of the other modes.
//
// modeRange(i) describes the reachable values of mode i with all other drivers held at the base pose:
//   closed     the driver winds round on the closed loop of the motion: one full period [0, 2 pi |winding|], the
//              pose repeats with that period
//   limited    both ends of the current arc are limits (extrema of the driver, or blocked): [tMin, tMax]. A driver
//              that merely swings on a closed configuration loop (rocker, slider, a chain link) is limited, never closed
//   half       one end is a limit; the other side runs to a fixed travel (FREE_TRAVEL, in driver units)
//   unbounded  no limit on either side: +-FREE_TRAVEL in driver units
// A side that neither blocks nor runs away within the search budget keeps the fixed travel too (status 'budget').

import { analyze, normalizeMode, pointVelocity, worldPoint, withFixedDrivers, bodyMotion, poleOf } from '../core/kinematics.js';
import { len } from '../core/geometry.js';
import { defaultSelection, listDrivers, validateDriverSelection, candidatesPerMode } from './drivers.js';
import { createCurve } from './curve.js';
import { searchBudget } from './curve-walk.js';
import { STEP, FREE_TRAVEL, CACHED_CURVES } from '../constants.js';
import { runSync, stepFor } from './task.js';


const FLAT_RATE = 1e-6; // |d driver / d distance| below this: the driver is flat along its mode here
const dotv = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const describeDriver = (d) => ({ id: d.id, kind: d.kind, label: d.label });

/**
 * Trajectory of the mechanism that `sys` forms at the pose `base`.
 * `options` = { drivers?: string[] (slot order), step?: number } (a plain number is the step, for older callers):
 * `drivers` selects the driver of each mode by id (see drivers.js); an invalid selection is reported in `warnings`
 * and the default one is used. For one degree of freedom a valid selection only picks the coordinate whose
 * winding and arcs modeRange reports; t stays the distance travelled by the fastest point.
 *   count            number of independent modes k (0 for a rigid system)
 *   modes            dq/dt of each mode at the base pose
 *   drivers          for k > 1: the driver of each mode { id, kind, body, label, value(q), gradient(q), index? }; [] otherwise
 *   slots            ids of the driver of each mode, in slot order (also for k = 1; [] when no driver moves)
 *   candidates       per mode, the drivers that could take its place { id, kind, label }[][], in canonical order (the
 *                    selection stays independent; the slot's own driver included)
 *   warnings         strings: what was ignored or defaulted (invalid driver selection)
 *   poseAt(t, opts?) { q, params, blocked }: pose for the parameters t (array of k numbers: the change of each mode's
 *                    driver from its value at the base pose, in driver units; composed in mode order, see above).
 *                    A value beyond the end of the current arc is clamped to the arc's end (the extremum of the
 *                    driver) with `blocked` = true; `params` holds the changes actually reached. Walks are cut off at
 *                    the walk limit (distance of the fastest point).
 *                    opts.arcs (k = 1 only) = [arc index]: stay on that arc of the driver (modeRange(0).arcs) instead
 *                    of the arc of the base pose, for playback that has moved on to an adjacent arc; opts.deadSense
 *                    (+1 / -1) picks the sense of the walk when the base pose is a dead point of the driver.
 *   candidateKinds(i) { [candidate id]: 'full' | 'partial' | 'unbounded' }: how each candidate of mode i would move (see below)
 *   rangeStep(i, ms) { done, range }: the classification of modeRange(i) advanced for about `ms` milliseconds per call, so
 *                    a long one can be sliced (modeRange(i) finishes whatever was started). kindsStep(i, ms) does the
 *                    same for candidateKinds(i) and returns { done, kinds }.
 *   modeRange(i)     classification of mode i alone (other drivers at the base pose):
 *                    { kind: 'closed' | 'limited' | 'half' | 'unbounded', tMin, tMax, period, angle,
 *                      freeMin, freeMax, capped, searched, status, sides, winding, driver, driverKind, extent,
 *                      arcs, branch, loopLength, value }
 *                    tMin <= 0 <= tMax (changes of the driver) except for closed modes, which span [0, period] with
 *                    period = 2 pi |winding|. `value` = the driver at the base pose (v = value + t). `angle` =
 *                    { body, sign: 1 } when a closed mode turns the body of its rotation driver exactly once round
 *                    (readable as 0..360 degrees); freeMin/freeMax flag the sides that end at the fixed travel
 *                    rather than a limit. `status` 'closed' | 'blocked' | 'free' | 'budget' tells how the curve was
 *                    established ('budget' = not fully explored; 'closed' = the configuration loop closes, of length
 *                    `loopLength`, which says nothing about the driver winding); `capped` (status free or budget) is
 *                    kept for older callers. `driver`, `winding`, `extent` ({ min, max } of the driver along the
 *                    curve), `arcs` (monotone pieces { from, to, gFrom, gTo, dir }, from/to = distance along the
 *                    curve relative to the base pose, scaled like the mode for k > 1) and `branch` (the arc that
 *                    holds the base pose, the one poseAt stays on) describe the driver on the curve; `driverKind` is
 *                    how the driver itself moves: 'closed' (winds round), 'limited' (swings between two extremes,
 *                    also on a closed configuration loop), 'half' or 'unbounded'.
 */
export function createTrajectory(sys, base, options = {}) {
  const opts = typeof options === 'number' ? { step: options } : options || {};
  const a = analyze(sys, base);
  const count = a.nullity;
  const all = listDrivers(sys);
  const warnings = [];

  let chosen = null;
  // A one-degree-of-freedom selection that fails only because the driver is flat at the base pose (a slider at its
  // stroke limit) is judged after the curve is known: it is accepted when the driver changes along the curve at all.
  let tentative = null;
  if (opts.drivers !== undefined && opts.drivers !== null && count > 0) {
    const checked = validateDriverSelection(sys, base, opts.drivers, a.modes);
    if (checked.ok) chosen = checked.drivers.map((d) => all.find((c) => c.id === d.id));
    else {
      const flat = count === 1 && Array.isArray(opts.drivers) && opts.drivers.length === 1 ? all.find((c) => c.id === opts.drivers[0]) : null;
      if (flat) tentative = { driver: flat, reason: checked.reason };
      else warnings.push(`driver selection ignored: ${checked.reason}`);
    }
  }
  // The default driver(s): the canonical-first independent ones (drivers.js), whatever the pose or the curve.
  const defaultDrivers = () => {
    const found = defaultSelection(sys, base, a.modes, all);
    if (!found && count > 1) throw new Error(`cannot choose ${count} independent driver coordinates for this mechanism`);
    return found;
  };
  if (count > 0 && !chosen && !tentative) chosen = defaultDrivers();
  const drivers = count > 1 ? chosen : [];
  const budget = searchBudget(sys, base, opts.step ?? STEP);
  const candidates = count ? candidatesPerMode(sys, base, a.modes, chosen || [], all).map((list) => list.map(describeDriver)) : [];

  /** System in which every driver except mode i's is held at its value in q. */
  const heldSystem = (i, q) => withFixedDrivers(sys, drivers.filter((_, j) => j !== i), q);

  /** Curve of mode i through pose q (other drivers held at their values in q); `scale` = distance per unit of t. */
  function curveAt(i, q) {
    if (count === 1) {
      const unit = normalizeMode(sys, q, a.modes[0]) || a.modes[0];
      return { curve: createCurve(sys, { q, tangent: a.modes[0] }, budget), scale: 1, unit };
    }
    const held = heldSystem(i, q);
    const found = analyze(held, q);
    if (!found.modes.length) throw new Error(`mode ${i + 1} has no motion when the other drivers are held`);
    const gradient = drivers[i].gradient(q);
    const tangent = dotv(gradient, found.modes[0]) < 0 ? found.modes[0].map((x) => -x) : found.modes[0];
    const unit = normalizeMode(held, q, tangent);
    const rate = unit ? dotv(gradient, unit) : 0;
    // A driver that is momentarily flat along its mode (a dead point: the selection is a function of the mechanism, not of
    // the pose, so it can sit on one) keeps the curve distance as the mode's unit; solveDriver handles the dead start.
    const flat = Math.abs(rate) < FLAT_RATE;
    return { curve: createCurve(held, { q, tangent }, budget), scale: flat ? 1 : 1 / rate, unit };
  }

  const primaries = a.modes.map((_, i) => curveAt(i, base));
  const preclassified = new Map();
  if (tentative) {
    const c = primaries[0].curve.classify({ driver: tentative.driver, candidates: all });
    if (c.extent && c.extent.max - c.extent.min > 1e-6) {
      chosen = [tentative.driver];
      preclassified.set(0, c);
    } else {
      warnings.push(`driver selection ignored: ${tentative.reason}`);
      chosen = defaultDrivers();
    }
  }
  const modes = primaries.map((p) => p.unit.map((x) => x * p.scale));
  const detached = new Map(); // curves started at other poses (a mode after earlier modes moved), oldest first
  const ranges = new Map();

  function detachedCurve(i, q) {
    const key = `${i}|${q.join(',')}`;
    let found = detached.get(key);
    if (!found) {
      found = curveAt(i, q);
      if (detached.size >= CACHED_CURVES) detached.delete(detached.keys().next().value);
      detached.set(key, found);
    }
    return found;
  }

  const resolved = []; // k = 1: the driver chosen by classification when none was selected
  const driverOf = (i) => {
    if (count > 1) return drivers[i];
    if (chosen) return chosen[0];
    modeRange(i);
    return resolved[i];
  };
  const wrap = (value, period) => ((value % period) + period) % period;

  function poseAt(t, options = {}) {
    let q = base;
    let atBase = true;
    let blocked = false;
    const params = new Array(count).fill(0);
    const arcOf = count === 1 && options.arcs ? options.arcs[0] : undefined;
    const arcDriver = arcOf !== undefined ? driverOf(0) : null;
    const arc = arcDriver && modeRange(0).kind === 'limited' ? modeRange(0).arcs[arcOf] : null;
    if (arc) {
      // The playback names the arc the mode is on: stay on it (also when it is not the arc of the base pose).
      const moved = primaries[0].curve.solveOnArc(arcDriver, arc, modeRange(0).value + t[0]);
      return { q: moved.q.slice(), params: [moved.delta], blocked: moved.blocked };
    }
    for (let i = 0; i < count; i++) {
      if (t[i] === 0) continue;
      const driver = driverOf(i);
      if (!driver) {
        blocked = true;
        continue;
      }
      const range = modeRange(i);
      const { curve } = atBase ? primaries[i] : detachedCurve(i, q);
      // A winding driver repeats its pose every period: within one turn the body angles are followed as they wind up,
      // beyond it the value is reduced (the same pose up to whole turns of the bodies), so long scrubs stay cheap.
      const periodic = range.kind === 'closed' && range.period > 0;
      const wrapped = periodic && Math.abs(t[i]) > range.period;
      const moved = curve.solveDriver(driver, wrapped ? wrap(t[i], range.period) : t[i], { deadSense: options.deadSense });
      q = moved.q;
      if (moved.delta !== 0) atBase = false;
      params[i] = wrapped ? t[i] : moved.delta;
      blocked = blocked || moved.blocked;
    }
    return { q: q.slice(), params, blocked };
  }

  const rangeJobs = new Map(); // mode -> running rangeSteps generator (shared by modeRange and rangeStep)
  function jobOf(jobs, i, make) {
    if (!jobs.has(i)) jobs.set(i, make(i));
    return jobs.get(i);
  }
  function modeRange(i) {
    return ranges.has(i) ? ranges.get(i) : runSync(jobOf(rangeJobs, i, rangeSteps));
  }
  function rangeStep(i, ms) {
    if (ranges.has(i)) return { done: true, range: ranges.get(i) };
    const step = stepFor(jobOf(rangeJobs, i, rangeSteps), ms);
    return { done: step.done, range: step.done ? step.value : null };
  }

  function* rangeSteps(i) {
    if (ranges.has(i)) return ranges.get(i);
    const { scale } = primaries[i];
    const selected = count > 1 ? drivers[i] : chosen && chosen[0];
    const c = preclassified.get(i) || (yield* primaries[i].curve.classifySteps({ driver: selected || undefined, candidates: all }));
    if (count === 1) resolved[i] = c.driver;
    const driver = c.driver;
    const describe = {
      status: c.status,
      sides: c.sides,
      winding: c.winding,
      driver: driver ? describeDriver(driver) : null,
      driverKind: c.driverKind,
      extent: c.extent,
      arcs: c.arcs.map((arc) => ({ ...arc, from: arc.from / scale, to: arc.to / scale })),
      branch: c.branch,
      searched: c.searched / scale,
      loopLength: c.status === 'closed' ? c.period / scale : null,
      value: driver ? driver.value(base) : 0,
    };
    const arc = c.branch >= 0 ? c.arcs[c.branch] : null;
    let out;
    if (!driver || !arc) {
      out = { kind: 'limited', tMin: 0, tMax: 0, period: 0, angle: null, freeMin: false, freeMax: false, capped: false, ...describe };
    } else if (c.status === 'closed' && c.winding !== 0 && c.arcs.length === 1) {
      const period = 2 * Math.PI * Math.abs(c.winding);
      const turnsOnce = driver.kind === 'rotation' && Math.abs(c.winding) === 1;
      // The parameter increases with the driver, so the angle of the rotated body is read with sign +1.
      out = {
        kind: 'closed', tMin: 0, tMax: period, period,
        angle: turnsOnce ? { body: driver.body, sign: 1 } : null,
        freeMin: false, freeMax: false, capped: false, ...describe,
      };
    } else {
      // Reachable values = the monotone arc that holds the base pose. An end is a limit when it is an extremum of the
      // driver or the curve is blocked there; the end of the explored curve without a block is open (fixed travel).
      const loop = c.status === 'closed';
      const first = c.arcs[0].from;
      const last = c.arcs[c.arcs.length - 1].to;
      const limitPos = loop || arc.to < last - 1e-9 || c.sides.pos === 'blocked';
      const limitNeg = loop || arc.from > first + 1e-9 || c.sides.neg === 'blocked';
      const up = arc.dir >= 0;
      const low = up ? { g: arc.gFrom, limit: limitNeg } : { g: arc.gTo, limit: limitPos };
      const high = up ? { g: arc.gTo, limit: limitPos } : { g: arc.gFrom, limit: limitNeg };
      const freeMin = !low.limit;
      const freeMax = !high.limit;
      out = {
        kind: !freeMin && !freeMax ? 'limited' : freeMin && freeMax ? 'unbounded' : 'half',
        tMin: freeMin ? -FREE_TRAVEL : Math.min(0, low.g - describe.value),
        tMax: freeMax ? FREE_TRAVEL : Math.max(0, high.g - describe.value),
        period: 0,
        angle: null,
        freeMin,
        freeMax,
        capped: c.status === 'free' || c.status === 'budget',
        ...describe,
      };
    }
    ranges.set(i, out);
    return out;
  }

  const candidateKindCache = new Map();
  /**
   * How each candidate driver of mode i moves along the mode's curve (other drivers held): { [id]: 'full' | 'partial' |
   * 'unbounded' }. 'full' = winds round (a crank); 'unbounded' = monotone on a curve without limits; 'partial' = swings
   * between extrema or is stopped (a slider, a rocker). Computed on first request (one classification per candidate).
   */
  const kindJobs = new Map();
  function* kindSteps(i) {
    if (candidateKindCache.has(i)) return candidateKindCache.get(i);
    const out = {};
    for (const entry of candidates[i] || []) {
      const c = yield* primaries[i].curve.classifySteps({ driver: all.find((d) => d.id === entry.id) });
      const monotone = c.arcs.length === 1 && (c.status === 'free' || c.status === 'budget');
      out[entry.id] = c.driverKind === 'closed' ? 'full' : monotone ? 'unbounded' : 'partial';
      yield;
    }
    candidateKindCache.set(i, out);
    return out;
  }
  const candidateKinds = (i) => (candidateKindCache.has(i) ? candidateKindCache.get(i) : runSync(jobOf(kindJobs, i, kindSteps)));
  function kindsStep(i, ms) {
    if (candidateKindCache.has(i)) return { done: true, kinds: candidateKindCache.get(i) };
    const step = stepFor(jobOf(kindJobs, i, kindSteps), ms);
    return { done: step.done, kinds: step.done ? step.value : null };
  }

  const slots = (chosen || []).map((d) => d.id);
  return { count, modes, drivers, slots, candidates, warnings, poseAt, modeRange, rangeStep, candidateKinds, kindsStep };
}

const fmt = (v) => (Math.abs(v) < 5e-3 ? 0 : v).toFixed(2).replace(/\.?0+$/, '');

/**
 * Short physical description of a mode at pose q: the body that moves most and how
 * ("body 2 rotates about (3, 7)" / "body 2 translates"). Bodies are numbered from 1.
 */
export function describeMode(sys, q, mode) {
  let best = -1;
  let bestSpeed = 0;
  sys.bodies.forEach((b, i) => {
    const speed = Math.max(...b.pts.map((p) => len(pointVelocity(q, mode, i, worldPoint(q, i, p)))));
    if (speed > bestSpeed + 1e-9) {
      bestSpeed = speed;
      best = i;
    }
  });
  if (best < 0) return 'no motion';
  const pole = poleOf(bodyMotion(q, mode, best));
  if (pole && pole.kind === 'point') return `body ${best + 1} rotates about (${fmt(pole.point.x)}, ${fmt(pole.point.y)})`;
  return `body ${best + 1} translates`;
}
