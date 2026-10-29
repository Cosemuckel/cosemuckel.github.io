// Deterministic finite motion along the mechanism's independent degrees of freedom (pure maths, no DOM).
//
// A mechanism with k = nullity(J) degrees of freedom is parametrised from a base pose by k numbers
// t = (t1..tk), one per mode.
//
// k = 1: the mode is the null-space vector at the base pose; t is the distance travelled by the fastest
// body point. The pose for t is found by walking from the base pose in fixed steps, each a tangent step
// followed by Newton correction. The tangent is tracked by projecting the previous one onto the null space
// of the current pose (continuity of the sense of motion, so dead points of a crank are passed).
//
// k > 1: an arbitrary null-space basis drifts with the pose and mixes the bodies, so modes are instead
// defined by natural DRIVER coordinates. At the base pose k generalised coordinates are chosen as drivers
// (pivoted elimination on the null-space rows; candidates in order: body rotation angles by body index,
// then body x / y translations), such that holding any k - 1 of them leaves exactly one degree of freedom.
//   mode i  = vary driver i while every other driver is held at its current value (extra `fix`
//             constraints): a true 1-DOF curve, walked exactly like the k = 1 case.
//   t_i     is in driver units (radians for a rotation, length for a translation): at the base pose
//             dt_i = d(driver i). Along the curve it stays proportional to the distance travelled by the
//             fastest body point (scale fixed at the base pose), so a bar that turns once round has period 2 pi.
// The pose for a vector t is the composition of the mode walks in mode order: first mode 1 is walked by t_1
// from the base pose, then mode 2 by t_2 from the pose reached (holding the then-current values of the
// other drivers, in particular the already moved driver 1), and so on. This is a pure function of
// (system, base pose, t): scrubbing to the same t always gives the same pose, however the user got there.
// Changing t_j never changes the driver coordinates of the other modes.
//
// modeRange(i) classifies the motion of one mode with all other drivers held at the base pose by walking
// the configuration-space curve through the base pose in both directions:
//   closed     the walk comes back to the base pose: one full period [0, P]; t = P is the pose of t = 0
//   limited    the walk is blocked in both directions: [tMin, tMax] between the two limits
//   half       blocked on one side only: that limit up to a fixed travel (FREE_TRAVEL) on the free side
//   unbounded  never blocked, never returns (a free roller, a pure translation): +-FREE_TRAVEL
// A walk that is neither blocked nor closed within the search budget is treated as free ("capped").

import { analyze, correct, normalizeMode, pointVelocity, worldPoint, withFixedCoordinates, bodyMotion, poleOf } from './kinematics.js';
import { len } from './geometry.js';

/** Parameter distance of one integration step (checkpoint spacing along a walk). */
export const STEP = 0.1;
/** Fixed travel offered on a side of a mode that has no limit (free roller, translation). */
export const FREE_TRAVEL = 12;
/** Longest walk (distance of the fastest point) explored per direction when classifying a mode; also the longest poseAt. */
export const MAX_WALK = 200;
const LEAVE = 0.5; // a walk counts as having left the base pose beyond this pose distance
const CLOSE = 0.25; // ... and as possibly returning when it comes closer than this again
const CLOSED_TOL = 1e-4; // refined return distance accepted as "back at the base pose"
const MAX_SPLIT = 3; // a failing step is halved at most this many times
const MIN_NORM = 1e-6;
const CACHED_CURVES = 8;
const DRIVER_TOLS = [1e-3, 1e-6]; // independence thresholds tried when picking drivers

const dotv = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

/** Projects the previous basis onto the current null space and re-orthonormalises; null if it degenerates. */
function trackBasis(sys, state) {
  const { modes } = analyze(sys, state.q);
  const out = [];
  for (const b of state.basis) {
    const w = new Array(b.length).fill(0);
    for (const m of modes) {
      const c = dotv(m, b);
      m.forEach((v, i) => (w[i] += c * v));
    }
    for (const u of out) {
      const c = dotv(w, u);
      u.forEach((v, i) => (w[i] -= c * v));
    }
    const l = Math.sqrt(dotv(w, w));
    if (l < MIN_NORM) return null;
    out.push(w.map((v) => v / l));
  }
  return out;
}

/** One step of parameter increment `delta` from `state` = { q, basis }; null when the step cannot be completed. */
function advance(sys, state, delta, depth = 0) {
  const basis = trackBasis(sys, state);
  if (!basis) return null;
  const d = new Array(state.q.length).fill(0);
  for (let i = 0; i < basis.length; i++) {
    if (delta[i] === 0) continue;
    const unit = normalizeMode(sys, state.q, basis[i]);
    if (!unit) return null;
    unit.forEach((v, j) => (d[j] += delta[i] * v));
  }
  const trial = correct(sys, state.q.map((v, j) => v + d[j]));
  if (trial.ok) return { q: trial.q, basis };
  if (depth >= MAX_SPLIT) return null;
  const half = delta.map((x) => x / 2);
  const mid = advance(sys, { q: state.q, basis }, half, depth + 1);
  return mid && advance(sys, mid, half, depth + 1);
}

/** Largest displacement of any body point between two poses (invariant under whole turns of a body). */
function poseDistance(sys, qa, qb) {
  let m = 0;
  sys.bodies.forEach((b, i) => {
    for (const p of b.pts) {
      const a = worldPoint(qa, i, p);
      const c = worldPoint(qb, i, p);
      m = Math.max(m, Math.hypot(a.x - c.x, a.y - c.y));
    }
  });
  return m;
}

/**
 * Picks k driver coordinates from the null-space basis `modes` (k vectors): the first candidates, in
 * order (rotation of body 0, 1, ..; then x, y of body 0, 1, ..), whose rows of the basis are linearly
 * independent of the rows already chosen. Holding any k - 1 of the drivers then leaves one degree of
 * freedom in which the remaining driver moves.
 */
function pickDrivers(sys, modes) {
  const k = modes.length;
  const candidates = [];
  for (let b = 0; b < sys.n; b++) candidates.push({ index: 3 * b + 2, body: b, kind: 'rotation', label: `rotation of body ${b + 1}` });
  for (let b = 0; b < sys.n; b++) {
    candidates.push({ index: 3 * b, body: b, kind: 'x', label: `translation of body ${b + 1} in x` });
    candidates.push({ index: 3 * b + 1, body: b, kind: 'y', label: `translation of body ${b + 1} in y` });
  }
  for (const tol of DRIVER_TOLS) {
    const chosen = [];
    const ortho = [];
    for (const cand of candidates) {
      const w = modes.map((m) => m[cand.index]);
      for (const u of ortho) {
        const c = dotv(w, u);
        u.forEach((v, i) => (w[i] -= c * v));
      }
      const l = Math.sqrt(dotv(w, w));
      if (l <= tol) continue;
      ortho.push(w.map((v) => v / l));
      chosen.push(cand);
      if (chosen.length === k) return chosen;
    }
  }
  throw new Error(`cannot choose ${k} independent driver coordinates for this mechanism`);
}

/**
 * One-degree-of-freedom curve of `sys` through `start` = { q, tangent } (a null-space vector there, whose
 * sign is the positive sense). All lengths are distances s travelled by the fastest body point.
 *   poseAt(s)   { q, reached, blocked }: pose at signed distance s; when blocked first (limit / singular
 *               position, or beyond MAX_WALK) q is the last reachable pose and `reached` the distance reached
 *   classify()  { period, angle, reachNeg, reachPos, searched }: period (and the body that turns once
 *               round, if any) for a curve that comes back to its start, null otherwise; reachNeg / reachPos
 *               are the limit distances on each side, null where there is no limit
 */
function createCurve(sys, start, searchLength, step) {
  const origin = { q: start.q.slice(), basis: [start.tangent.slice()] };
  const dq0 = normalizeMode(sys, start.q, start.tangent) || start.tangent;
  const maxSteps = Math.round(searchLength / step);
  const walks = new Map([1, -1].map((sign) => [sign, { sign, states: [origin], blocked: false }]));

  const extend = (walk, n) => {
    while (!walk.blocked && walk.states.length <= n) {
      const next = advance(sys, walk.states[walk.states.length - 1], [walk.sign * step]);
      if (next) walk.states.push(next);
      else walk.blocked = true;
    }
  };

  function poseAt(s) {
    const full = Math.abs(s);
    const L = Math.min(full, MAX_WALK); // longer walks are cut off (reported as blocked)
    if (L < 1e-12) return { q: start.q.slice(), reached: 0, blocked: false };
    const walk = walks.get(s < 0 ? -1 : 1);
    const N = Math.floor(L / step + 1e-9);
    extend(walk, N);
    const good = walk.states.length - 1;
    const at = (n, dist, blocked) => ({ q: walk.states[n].q.slice(), reached: walk.sign * dist, blocked });
    if (good < N) return at(good, good * step, true);
    const rem = L - N * step;
    if (rem < 1e-12) return at(N, N * step, L < full - 1e-12);
    const next = advance(sys, walk.states[N], [walk.sign * rem]);
    if (!next) return at(N, N * step, true);
    return { q: next.q, reached: walk.sign * L, blocked: L < full - 1e-12 };
  }

  /** Distance travelled before the first checkpoint the walk cannot reach (halving the last step 14 times). */
  function reachOfBlockedWalk(walk) {
    const good = walk.states.length - 1;
    let lo = 0;
    let hi = step;
    for (let k = 0; k < 14; k++) {
      const mid = (lo + hi) / 2;
      if (advance(sys, walk.states[good], [walk.sign * mid])) lo = mid;
      else hi = mid;
    }
    // Back off a hair so that poseAt(reach) (one single step) is certain to succeed.
    return good * step + Math.max(0, lo - 1e-4);
  }

  /** Distance at which the walk is back at the start pose, searched around checkpoint c; null if it is not. */
  function refineReturn(walk, c) {
    let best = { d: Infinity, at: 0 };
    for (const m of [c - 1, c]) {
      if (m < 0 || m >= walk.states.length) continue;
      const distanceAt = (h) => {
        const s = advance(sys, walk.states[m], [walk.sign * h]);
        return s ? poseDistance(sys, start.q, s.q) : Infinity;
      };
      let lo = 0;
      let hi = step;
      for (let k = 0; k < 45; k++) {
        const m1 = lo + (hi - lo) / 3;
        const m2 = hi - (hi - lo) / 3;
        if (distanceAt(m1) < distanceAt(m2)) hi = m2;
        else lo = m1;
      }
      const h = (lo + hi) / 2;
      const d = distanceAt(h);
      if (d < best.d) best = { d, at: m * step + h };
    }
    return best.d < CLOSED_TOL ? best.at : null;
  }

  /** Walks in one direction: { type: 'closed', period } | { type: 'blocked', reach } | { type: 'free' }. */
  function scan(sign, detectReturn) {
    const walk = walks.get(sign);
    const dist = [0];
    let left = false;
    for (let n = 1; n <= maxSteps; n++) {
      extend(walk, n);
      if (walk.states.length - 1 < n) return { type: 'blocked', reach: reachOfBlockedWalk(walk) };
      dist.push(poseDistance(sys, start.q, walk.states[n].q));
      if (dist[n] > LEAVE) left = true;
      // dist[n - 1] was a local minimum near the start pose: the walk may be coming back round.
      if (detectReturn && left && n >= 2 && dist[n - 1] < CLOSE && dist[n] >= dist[n - 1]) {
        const period = refineReturn(walk, n - 1);
        if (period !== null) return { type: 'closed', period };
      }
    }
    return { type: 'free' };
  }

  /** The body that makes exactly one whole turn over a closed curve (fastest first), or null. */
  function turnedBody(period) {
    const end = poseAt(period).q;
    let best = null;
    let bestSpeed = 0;
    sys.bodies.forEach((b, k) => {
      const turn = end[3 * k + 2] - start.q[3 * k + 2];
      if (Math.abs(Math.abs(turn) - 2 * Math.PI) > 0.02) return;
      const speed = Math.max(...b.pts.map((p) => len(pointVelocity(start.q, dq0, k, worldPoint(start.q, k, p)))));
      if (!best || speed > bestSpeed) {
        best = { body: k, sign: Math.sign(turn) };
        bestSpeed = speed;
      }
    });
    return best;
  }

  function classify() {
    const searched = maxSteps * step;
    const pos = scan(1, true);
    if (pos.type === 'closed') return { period: pos.period, angle: turnedBody(pos.period), reachNeg: null, reachPos: null, searched };
    const neg = scan(-1, false);
    return {
      period: null,
      angle: null,
      reachNeg: neg.type === 'blocked' ? neg.reach : null,
      reachPos: pos.type === 'blocked' ? pos.reach : null,
      searched,
    };
  }

  return { poseAt, classify };
}

/**
 * Trajectory of the mechanism that `sys` forms at the pose `base`.
 *   count            number of independent modes k (0 for a rigid system)
 *   modes            dq/dt of each mode at the base pose
 *   drivers          for k > 1: the driver coordinate of each mode { index, body, kind, label }; [] otherwise
 *   poseAt(t)        { q, params, blocked }: pose for the parameters t (array of k numbers, composed in mode
 *                    order, see above); when a mode is blocked first (limit / singular position) its walk ends
 *                    at the last reachable pose and `params` holds the parameters actually reached;
 *                    walks are cut off at MAX_WALK (distance of the fastest point)
 *   modeRange(i)     classification of mode i alone (other drivers at the base pose):
 *                    { kind: 'closed' | 'limited' | 'half' | 'unbounded', tMin, tMax, period, angle,
 *                      freeMin, freeMax, capped, searched }
 *                    tMin <= 0 <= tMax except for closed modes, which span [0, period]. `angle` =
 *                    { body, sign } when a closed mode turns that body once round (readable as 0..360 degrees);
 *                    freeMin/freeMax flag the sides that end at the fixed travel rather than a limit;
 *                    `capped` = a direction found neither a limit nor a return within `searched` (the
 *                    search budget: MAX_WALK, shared between the modes).
 */
export function createTrajectory(sys, base, step = STEP) {
  const a = analyze(sys, base);
  const count = a.nullity;
  const drivers = count > 1 ? pickDrivers(sys, a.modes) : [];
  const searchLength = MAX_WALK / Math.max(1, count);

  /** System in which every driver except mode i's is held at its value in q. */
  const heldSystem = (i, q) => withFixedCoordinates(sys, drivers.filter((_, j) => j !== i).map((d) => d.index), q);

  /** Curve of mode i through pose q (other drivers held at their values in q); `scale` = distance per unit of t. */
  function curveAt(i, q) {
    if (count === 1) {
      const unit = normalizeMode(sys, q, a.modes[0]) || a.modes[0];
      return { curve: createCurve(sys, { q, tangent: a.modes[0] }, searchLength, step), scale: 1, unit };
    }
    const held = heldSystem(i, q);
    const found = analyze(held, q);
    if (!found.modes.length) throw new Error(`mode ${i + 1} has no motion when the other drivers are held`);
    const index = drivers[i].index;
    const tangent = found.modes[0][index] < 0 ? found.modes[0].map((x) => -x) : found.modes[0];
    const unit = normalizeMode(held, q, tangent);
    if (!unit || Math.abs(unit[index]) < 1e-12) throw new Error(`the driver of mode ${i + 1} does not move with the mode`);
    return { curve: createCurve(held, { q, tangent }, searchLength, step), scale: 1 / unit[index], unit };
  }

  const primaries = a.modes.map((_, i) => curveAt(i, base));
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

  function poseAt(t) {
    let q = base;
    let atBase = true;
    let blocked = false;
    const params = new Array(count).fill(0);
    for (let i = 0; i < count; i++) {
      if (t[i] === 0) continue;
      const { curve, scale } = atBase ? primaries[i] : detachedCurve(i, q);
      const moved = curve.poseAt(t[i] * scale);
      q = moved.q;
      if (moved.reached !== 0) atBase = false;
      params[i] = moved.reached / scale;
      blocked = blocked || moved.blocked;
    }
    return { q: q.slice(), params, blocked };
  }

  function modeRange(i) {
    if (ranges.has(i)) return ranges.get(i);
    const { curve, scale } = primaries[i];
    const c = curve.classify();
    const searched = c.searched / scale;
    let out;
    if (c.period !== null) {
      const period = c.period / scale;
      out = { kind: 'closed', tMin: 0, tMax: period, period, angle: c.angle, freeMin: false, freeMax: false, capped: false, searched };
    } else {
      const blockedNeg = c.reachNeg !== null;
      const blockedPos = c.reachPos !== null;
      out = {
        kind: blockedPos && blockedNeg ? 'limited' : blockedPos || blockedNeg ? 'half' : 'unbounded',
        tMin: blockedNeg ? -c.reachNeg / scale : -FREE_TRAVEL,
        tMax: blockedPos ? c.reachPos / scale : FREE_TRAVEL,
        period: 0,
        angle: null,
        freeMin: !blockedNeg,
        freeMax: !blockedPos,
        capped: !blockedPos || !blockedNeg,
        searched,
      };
    }
    ranges.set(i, out);
    return out;
  }

  return { count, modes, drivers, poseAt, modeRange };
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

/**
 * One playback tick of the mode parameters. `state` = { values, dir (+1 / -1 per mode), playing,
 * locked, ranges } (arrays indexed by mode); `ranges[i]` = { min, max, kind } where kind 'closed' also
 * carries `period` = max - min. `ds` is the advance per tick (a number, or one number per mode).
 * Playing, unlocked modes advance and turn round at the ends of their range; locked modes keep their
 * value whatever else is happening. A closed mode wraps from one end to the other (a full revolution
 * loops) as long as it is the only mode moving: with several modes moving together the pose is only
 * defined along one straight walk in parameter space, so a closed mode then turns round like the others.
 */
export function advanceParams({ values, dir, playing, locked, ranges }, ds) {
  const nextValues = values.slice();
  const nextDir = dir.slice();
  const moving = values.filter((_, i) => playing[i] && !locked[i]).length;
  values.forEach((v, i) => {
    if (!playing[i] || locked[i]) return;
    const { min, max, kind } = ranges[i];
    const step = Array.isArray(ds) ? ds[i] : ds;
    let t = v + dir[i] * step;
    if (kind === 'closed' && moving === 1 && max > min) {
      const period = max - min;
      t = min + ((((t - min) % period) + period) % period);
    } else if (t >= max) {
      t = max;
      nextDir[i] = -1;
    } else if (t <= min) {
      t = min;
      nextDir[i] = 1;
    }
    nextValues[i] = t;
  });
  return { values: nextValues, dir: nextDir };
}

/**
 * Display coordinates of a mode: the user-facing value u, a continuous number whose zero is where the
 * mode was first shown; the trajectory is rebased at the current pose with `offset` = u there, so
 * t = u - offset. Closed modes use u in [0, period] and t is u - offset taken modulo the period.
 */
export function paramFromValue(range, offset, u) {
  if (range.kind !== 'closed') return u - offset;
  const period = range.max - range.min;
  return (((u - offset) % period) + period) % period;
}

/** Inverse of paramFromValue for a pose reached at parameter t. */
export function valueFromParam(range, offset, t) {
  if (range.kind !== 'closed') return offset + t;
  const period = range.max - range.min;
  return range.min + ((((offset + t - range.min) % period) + period) % period);
}

/**
 * Display range of mode i of a trajectory rebased at a pose where the mode is shown as `offset`.
 * `previous` (the display range before the rebase, if any) keeps the fixed travel of a free side
 * anchored, so rebasing at another pose does not slide the window.
 */
export function displayRange(info, offset, previous) {
  if (info.kind === 'closed') return { ...info, min: 0, max: info.period };
  const side = (free, limit, prevBound) => (free ? (prevBound !== undefined ? prevBound : offset + limit) : offset + limit);
  const reusable = previous && previous.kind !== 'closed';
  return {
    ...info,
    min: side(info.freeMin, info.tMin, reusable && previous.freeMin ? previous.min : undefined),
    max: side(info.freeMax, info.tMax, reusable && previous.freeMax ? previous.max : undefined),
  };
}
