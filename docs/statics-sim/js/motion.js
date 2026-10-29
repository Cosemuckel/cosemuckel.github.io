// Deterministic finite motion along the mechanism's independent degrees of freedom (pure maths, no DOM).
//
// A mechanism with k = nullity(J) degrees of freedom is parametrised from a base pose by k numbers
// t = (t1..tk): mode i is the i-th vector of an orthonormal null-space basis at the base pose, and one
// unit of t_i is the distance travelled by the fastest body point (so t has length units). The pose for
// t is found by walking from the base pose along the straight line 0 -> t in parameter space in fixed
// steps, each a tangent step along the tracked modes followed by Newton correction. The result is a
// pure function of (system, base pose, t): scrubbing to the same t always gives the same pose, however
// the user got there. The modes are tracked by projecting the previous basis onto the null space of
// the current pose (continuity of the sense of motion; the basis is re-orthonormalised each step).

import { analyze, correct, normalizeMode, bodyMotion, poleOf, pointVelocity, worldPoint } from './kinematics.js';
import { len } from './geometry.js';

/** Parameter distance of one integration step (checkpoint spacing along a walk). */
export const STEP = 0.1;
/** Largest |t| of a single mode that is explored when looking for the range of a mode. */
export const LIMIT = 12;
const MAX_SPLIT = 3; // a failing step is halved at most this many times
const MIN_NORM = 1e-6;
const CACHED_WALKS = 6;

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

/**
 * Trajectory of the mechanism that `sys` forms at the pose `base`.
 *   count            number of independent modes k (0 for a rigid system)
 *   modes            the unit-speed mode vectors at the base pose
 *   poseAt(t)        { q, params, blocked }: pose for the parameters t (array of k numbers); when the
 *                    motion is blocked first (limit / singular position) q is the last reachable pose along
 *                    the line 0 -> t and `params` the parameters actually reached
 *   range(i)         { min, max }: how far mode i alone can be driven either way from the base pose
 */
export function createTrajectory(sys, base, step = STEP) {
  const a = analyze(sys, base);
  const count = a.nullity;
  const start = { q: base.slice(), basis: a.modes.map((m) => m.slice()) };
  const modes = a.modes.map((m) => normalizeMode(sys, base, m) || m);
  const walks = new Map(); // direction key -> { u, states: checkpoints at 0, step, 2 step, ...; blocked }

  const walkAlong = (u) => {
    const key = u.map((x) => x.toFixed(9)).join(',');
    let walk = walks.get(key);
    if (!walk) {
      walk = { u, states: [start], blocked: false };
      if (walks.size >= CACHED_WALKS) walks.delete(walks.keys().next().value);
      walks.set(key, walk);
    }
    return walk;
  };
  const extend = (walk, n) => {
    while (!walk.blocked && walk.states.length <= n) {
      const next = advance(sys, walk.states[walk.states.length - 1], walk.u.map((x) => x * step));
      if (next) walk.states.push(next);
      else walk.blocked = true;
    }
  };

  function poseAt(t) {
    const full = Math.hypot(...t);
    const L = Math.min(full, LIMIT * Math.max(1, count)); // longer walks are cut off (reported as blocked)
    if (count === 0 || L < 1e-12) return { q: base.slice(), params: t.map(() => 0), blocked: false };
    const u = t.map((x) => x / full);
    const walk = walkAlong(u);
    const N = Math.floor(L / step + 1e-9);
    extend(walk, N);
    const good = walk.states.length - 1;
    const at = (n, dist, blocked) => ({ q: walk.states[n].q.slice(), params: u.map((x) => x * dist), blocked });
    if (good < N) return at(good, good * step, true);
    const rem = L - N * step;
    if (rem < 1e-12) return at(N, N * step, L < full - 1e-12);
    const next = advance(sys, walk.states[N], u.map((x) => x * rem));
    if (!next) return at(N, N * step, true);
    return { q: next.q, params: u.map((x) => x * L), blocked: L < full - 1e-12 };
  }

  function range(i) {
    const out = { min: 0, max: 0 };
    for (const sign of [1, -1]) {
      const u = new Array(count).fill(0);
      u[i] = sign;
      const walk = walkAlong(u);
      extend(walk, Math.floor(LIMIT / step));
      let reach = (walk.states.length - 1) * step;
      if (walk.blocked) {
        // Creep up to the limit with ever smaller steps.
        let state = walk.states[walk.states.length - 1];
        let h = step;
        for (let k = 0; k < 5; k++) {
          h /= 2;
          const next = advance(sys, state, u.map((x) => x * h));
          if (next) {
            state = next;
            reach += h;
          }
        }
      }
      if (sign > 0) out.max = reach;
      else out.min = -reach;
    }
    return out;
  }

  return { count, modes, poseAt, range };
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
 * locked, ranges } (arrays indexed by mode). Playing, unlocked modes advance by `ds` and turn round
 * at the ends of their range; locked modes keep their value whatever else is happening.
 */
export function advanceParams({ values, dir, playing, locked, ranges }, ds) {
  const nextValues = values.slice();
  const nextDir = dir.slice();
  values.forEach((v, i) => {
    if (!playing[i] || locked[i]) return;
    const { min, max } = ranges[i];
    let t = v + dir[i] * ds;
    if (t >= max) {
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
