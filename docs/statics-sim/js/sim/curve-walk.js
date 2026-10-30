// @ts-check
// Stepping along a configuration curve and the search budget (pure maths): the tracked tangent basis, one corrected
// step with step halving, pose distances and the budget derived from the mechanism. curve.js builds the walks, arcs
// and range classification on top of these.

import { analyze, correct, normalizeMode, pointVelocity, worldPoint } from '../core/kinematics.js';
import { len } from '../core/geometry.js';
import { STEP, SEARCH_FACTOR, MAX_TOTAL_STEPS } from '../constants.js';

const MAX_SPLIT = 3; // a failing step is halved at most this many times
export const MIN_NORM = 1e-6;

export const dotv = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

const trackedBases = new WeakMap(); // state -> { sys, basis }: the tracked basis of a state never changes

/**
 * Projects the previous basis onto the current null space and re-orthonormalises; null if it degenerates. A walk state
 * is advanced from many times (bisections, root finding), so its result is kept on the state object; states are never
 * modified after they are created.
 */
export function trackBasis(sys, state) {
  const known = trackedBases.get(state);
  if (known && known.sys === sys) return known.basis;
  const basis = projectBasis(sys, state);
  trackedBases.set(state, { sys, basis });
  return basis;
}

function projectBasis(sys, state) {
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
export function advance(sys, state, delta, depth = 0) {
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
export function poseDistance(sys, qa, qb) {
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

/** Diagonal of the bounding box of all body points at pose q (0 for an empty system). */
export function boundingDiagonal(sys, q) {
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  sys.bodies.forEach((b, i) => {
    for (const p of b.pts) {
      const P = worldPoint(q, i, p);
      x0 = Math.min(x0, P.x);
      x1 = Math.max(x1, P.x);
      y0 = Math.min(y0, P.y);
      y1 = Math.max(y1, P.y);
    }
  });
  return x1 >= x0 ? Math.hypot(x1 - x0, y1 - y0) : 0;
}

/**
 * Search budget derived from the mechanism: each direction explores SEARCH_FACTOR x the bounding diagonal, in
 * steps of `step` x max(1, diagonal / 10) (so long bars are not walked in tiny steps), at most MAX_TOTAL_STEPS steps
 * for both directions together. `scale` is the step scale, also applied to the closeness thresholds of the search;
 * `diagonal` (at least 1) is the size the walk must clearly exceed before it is called free.
 */
export function searchBudget(sys, q, step = STEP) {
  const diagonal = Math.max(boundingDiagonal(sys, q), 1);
  const scale = Math.max(1, diagonal / 10);
  const searchLength = SEARCH_FACTOR * diagonal;
  let h = step * scale;
  if ((2 * searchLength) / h > MAX_TOTAL_STEPS) h = (2 * searchLength) / MAX_TOTAL_STEPS;
  return { searchLength, step: h, maxSteps: Math.max(1, Math.round(searchLength / h)), scale, diagonal };
}

/** Largest point speed of body k for the motion dq at pose q (used to prefer the body that moves most). */
export function bodySpeed(q, dq, k, body) {
  return Math.max(...body.pts.map((p) => len(pointVelocity(q, dq, k, worldPoint(q, k, p)))));
}
