// Planar rigid-body kinematics (pure math, no DOM).
//
// Generalised coordinates: every body i has pose q[3i..3i+2] = (tx, ty, theta) and a
// reference-point p moves to  P = R(theta) p + t.  The reference configuration is q = 0.
// Every support contributes scalar constraint equations r(q) = 0:
//   pin    : P(p) - p = 0                       (2 equations; ground point = p)
//   clamp  : theta = 0                          (1, added for clamped pins)
//   slide  : n . (P(p) - p) = 0, n = unit normal of the rail   (1; roller)
//   joint  : P_a(p) - P_b(p) = 0                (2 per extra body at the point)
// A locked roller is treated as a pin.
//
// Infinitesimal motions dq satisfy J(q) dq = 0, J = dr/dq, so they span the null space of
// J. A body point P then moves with velocity  v = dt + dtheta * perp(P - t): a rigid
// rotation about the body's pole (instantaneous centre) or, if dtheta = 0, a translation
// (pole at infinity). Finite motion is followed with tangent steps + Newton correction.

import { perp, unit, fromAngleDeg, intersectLines, add, sub, mul, len, dist } from './geometry.js';
import { nullspace, solve } from './linalg.js';
import { bodiesAtPoint } from './model.js';
import { pairsLabel } from './kennedy.js';

const EQ_COUNT = { pin: 2, clamp: 1, slide: 1, joint: 2 };
const NULL_TOL = 1e-7;
const CORRECT_TOL = 1e-10;

// ------------------------------------------------------------------ system

/**
 * Derive the constraint system from a scene. Supports that do not touch enough bodies
 * are inactive (reported through supportInfo, ignored by the maths).
 */
export function buildSystem(scene) {
  const bodies = scene.bodies.map((b) => ({ id: b.id, kind: b.kind, pts: b.pts, segs: b.segs }));
  const index = new Map(bodies.map((b, i) => [b.id, i]));
  const constraints = [];
  const supportInfo = new Map();

  for (const s of scene.supports) {
    const at = bodiesAtPoint(scene, s.p).map((b) => index.get(b.id));
    const info = { active: false, body: -1, bodies: at };
    supportInfo.set(s.id, info);
    if (s.type === 'joint') {
      if (at.length < 2) continue;
      for (let k = 1; k < at.length; k++) {
        constraints.push({ kind: 'joint', support: s.id, a: at[0], b: at[k], p: s.p });
      }
      info.active = true;
      info.body = at[0];
      continue;
    }
    if (at.length < 1) continue;
    const body = at[at.length - 1]; // topmost body
    info.active = true;
    info.body = body;
    if (s.type === 'pin' || (s.type === 'roller' && s.locked)) {
      constraints.push({ kind: 'pin', support: s.id, body, p: s.p });
      if (s.type === 'pin' && s.clamped) constraints.push({ kind: 'clamp', support: s.id, body });
    } else {
      const n = perp(fromAngleDeg(s.angleDeg));
      constraints.push({ kind: 'slide', support: s.id, body, p: s.p, n });
    }
  }
  const eq = constraints.reduce((sum, c) => sum + EQ_COUNT[c.kind], 0);
  return { n: bodies.length, bodies, constraints, supportInfo, eq };
}

export const zeroState = (sys) => new Array(3 * sys.n).fill(0);

/** World position of body-fixed reference point p. */
export function worldPoint(q, i, p) {
  const th = q[3 * i + 2];
  const c = Math.cos(th);
  const s = Math.sin(th);
  return { x: q[3 * i] + c * p.x - s * p.y, y: q[3 * i + 1] + s * p.x + c * p.y };
}

/** Inverse of worldPoint: body-fixed reference coordinates of a world point. */
export function bodyPoint(q, i, P) {
  const th = q[3 * i + 2];
  const c = Math.cos(th);
  const s = Math.sin(th);
  const dx = P.x - q[3 * i];
  const dy = P.y - q[3 * i + 1];
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
}

/** d/dtheta of worldPoint = perp(R p). */
function worldPointDTheta(q, i, p) {
  const th = q[3 * i + 2];
  const c = Math.cos(th);
  const s = Math.sin(th);
  return { x: -s * p.x - c * p.y, y: c * p.x - s * p.y };
}

/** Residuals r(q) and Jacobian J(q) (rows = equations, columns = 3 per body). */
export function evaluate(sys, q) {
  const cols = 3 * sys.n;
  const r = [];
  const J = [];
  const newRow = () => {
    const row = new Array(cols).fill(0);
    J.push(row);
    return row;
  };
  for (const c of sys.constraints) {
    if (c.kind === 'pin') {
      const P = worldPoint(q, c.body, c.p);
      const dP = worldPointDTheta(q, c.body, c.p);
      const j = 3 * c.body;
      const r0 = newRow();
      const r1 = newRow();
      r0[j] = 1;
      r1[j + 1] = 1;
      r0[j + 2] = dP.x;
      r1[j + 2] = dP.y;
      r.push(P.x - c.p.x, P.y - c.p.y);
    } else if (c.kind === 'clamp') {
      newRow()[3 * c.body + 2] = 1;
      r.push(q[3 * c.body + 2]);
    } else if (c.kind === 'slide') {
      const P = worldPoint(q, c.body, c.p);
      const dP = worldPointDTheta(q, c.body, c.p);
      const j = 3 * c.body;
      const row = newRow();
      row[j] = c.n.x;
      row[j + 1] = c.n.y;
      row[j + 2] = c.n.x * dP.x + c.n.y * dP.y;
      r.push(c.n.x * (P.x - c.p.x) + c.n.y * (P.y - c.p.y));
    } else {
      const Pa = worldPoint(q, c.a, c.p);
      const Pb = worldPoint(q, c.b, c.p);
      const dA = worldPointDTheta(q, c.a, c.p);
      const dB = worldPointDTheta(q, c.b, c.p);
      const ja = 3 * c.a;
      const jb = 3 * c.b;
      const r0 = newRow();
      const r1 = newRow();
      r0[ja] = 1;
      r1[ja + 1] = 1;
      r0[jb] = -1;
      r1[jb + 1] = -1;
      r0[ja + 2] = dA.x;
      r1[ja + 2] = dA.y;
      r0[jb + 2] = -dB.x;
      r1[jb + 2] = -dB.y;
      r.push(Pa.x - Pb.x, Pa.y - Pb.y);
    }
  }
  return { r, J };
}

// ---------------------------------------------------------------- analysis

/**
 * Degree-of-freedom analysis at configuration q.
 *  grubler   : 3n - equations (counting formula, ignores geometry)
 *  nullity   : dimension of the infinitesimal motion space (true instantaneous mobility)
 *  redundant : equations - rank(J)  (> 0 means statically indeterminate)
 *  modes     : orthonormal basis of the infinitesimal motions
 */
export function analyze(sys, q) {
  const { J } = evaluate(sys, q);
  const modes = nullspace(J, 3 * sys.n, NULL_TOL);
  const nullity = modes.length;
  const rank = 3 * sys.n - nullity;
  const redundant = sys.eq - rank;
  const grubler = 3 * sys.n - sys.eq;
  let kind;
  if (sys.n === 0) kind = 'empty';
  else if (nullity > 0) kind = 'mechanism';
  else kind = redundant === 0 ? 'determinate' : 'indeterminate';
  return { n: sys.n, eq: sys.eq, grubler, nullity, redundant, kind, modes };
}

export function describe(a) {
  switch (a.kind) {
    case 'empty':
      return { short: 'No bodies', long: 'Draw a body (a line) to start.' };
    case 'determinate':
      return { short: 'Rigid - statically determinate', long: 'No admissible motion; as many independent constraints as unknowns.' };
    case 'indeterminate':
      return {
        short: `Rigid - ${a.redundant}x statically indeterminate`,
        long: `No admissible motion; ${a.redundant} redundant constraint equation(s).`,
      };
    default: {
      const special = a.nullity > Math.max(a.grubler, 0);
      return {
        short: `Mechanism - ${a.nullity} DOF`,
        long:
          `Admissible infinitesimal motion with ${a.nullity} degree(s) of freedom.` +
          (special ? ' More than the Grubler count: special geometry (redundant constraints or infinitesimal mechanism).' : ''),
      };
    }
  }
}

// -------------------------------------------------------- velocity fields

/** Velocity of the world point P when rigidly attached to body i, for motion dq at pose q. */
export function pointVelocity(q, dq, i, P) {
  const rx = P.x - q[3 * i];
  const ry = P.y - q[3 * i + 1];
  const w = dq[3 * i + 2];
  return { x: dq[3 * i] - w * ry, y: dq[3 * i + 1] + w * rx };
}

/** Body motion as v(P) = v0 + omega * perp(P), i.e. relative to the world origin. */
export function bodyMotion(q, dq, i) {
  const w = dq[3 * i + 2];
  return {
    v0: { x: dq[3 * i] + w * q[3 * i + 1], y: dq[3 * i + 1] - w * q[3 * i] },
    omega: w,
  };
}

/**
 * Pole (instantaneous centre) of a body motion:
 *   { kind: 'point', point }          where v = 0
 *   { kind: 'infinity', direction }   pure translation; pole lies at infinity perpendicular to v
 *   null                              body at rest
 */
export function poleOf(motion, eps = 1e-9) {
  const { v0, omega } = motion;
  if (Math.abs(omega) > eps) return { kind: 'point', point: { x: -v0.y / omega, y: v0.x / omega } };
  if (len(v0) > eps) return { kind: 'infinity', direction: unit(perp(v0)) };
  return null;
}

/** Largest point speed over all body points; used to scale a mode to unit displacement. */
function maxSpeed(sys, q, dq) {
  let m = 0;
  sys.bodies.forEach((b, i) => {
    for (const p of b.pts) m = Math.max(m, len(pointVelocity(q, dq, i, worldPoint(q, i, p))));
  });
  return m;
}

/** Scale dq so that the fastest body point has speed 1 (returns null for a zero mode). */
export function normalizeMode(sys, q, dq) {
  const m = maxSpeed(sys, q, dq);
  return m < 1e-12 ? null : dq.map((x) => x / m);
}

// --------------------------------------------------------- pole beams

/** Current world position of a support's point (on its attached body). */
export function supportPoint(sys, q, s) {
  const info = sys.supportInfo.get(s.id);
  return info && info.active ? worldPoint(q, info.body, s.p) : s.p;
}

/**
 * Pole items per active support:
 *   roller (free)    : 'line'  - the pole beam through the support, perpendicular to the rail
 *   pin / locked / joint : 'point' - the pole itself (absolute pole for pins, relative pole for joints)
 * Clamped pins have no pole (the body cannot move at all).
 * Every item carries `pairs`, the relative poles (i, j) it belongs to (bodies numbered 1.., ground 0):
 * a pin / locked roller on body k is the pole (0, k), a roller beam is a line through (0, k), a joint
 * of bodies a, b, c.. is the common point of (a,b), (a,c), (b,c)... `label` is that notation as text.
 */
export function poleItems(sys, q, scene) {
  const items = [];
  for (const s of scene.supports) {
    const info = sys.supportInfo.get(s.id);
    if (!info || !info.active) continue;
    const point = supportPoint(sys, q, s);
    const base = { id: s.id, type: s.type, point, showBeam: s.showBeam };
    const top = info.body + 1; // body numbers are 1-based, ground is 0
    const item = (extra, pairs) => items.push({ ...base, ...extra, pairs, label: pairsLabel(pairs) });
    if (s.type === 'roller' && !s.locked) {
      item({ kind: 'line', dir: perp(fromAngleDeg(s.angleDeg)) }, [[0, top]]);
    } else if (s.type === 'joint') {
      const numbers = info.bodies.map((i) => i + 1).sort((a, b) => a - b);
      item({ kind: 'point' }, numbers.flatMap((a, k) => numbers.slice(k + 1).map((b) => [a, b])));
    } else if (!(s.type === 'pin' && s.clamped)) {
      item({ kind: 'point' }, [[0, top]]);
    }
  }
  return items;
}

/**
 * Pairwise intersections of the selected pole beams (lines). Parallel beams yield
 * { kind: 'parallel', direction } = intersection at infinity. `pair` is the relative pole
 * [0, k] when both beams belong to the same body k (the intersection is then its absolute
 * pole), otherwise null.
 */
export function selectedIntersections(items, selectedIds) {
  const lines = items.filter((it) => it.kind === 'line' && selectedIds.has(it.id));
  const out = [];
  for (let i = 0; i < lines.length; i++) {
    for (let j = i + 1; j < lines.length; j++) {
      const a = lines[i];
      const b = lines[j];
      const result = intersectLines({ p: a.point, d: a.dir }, { p: b.point, d: b.dir });
      const sameBody = a.pairs[0][1] === b.pairs[0][1];
      out.push({ a: a.id, b: b.id, anchor: mul(add(a.point, b.point), 0.5), result, pair: sameBody ? a.pairs[0] : null });
    }
  }
  return out;
}

// -------------------------------------------------------------- finite motion

/**
 * Newton projection back onto the constraint manifold (minimum-norm, damped so that
 * redundant constraints do not make the normal equations singular).
 */
export function correct(sys, q0, maxIter = 30) {
  let q = q0.slice();
  for (let it = 0; it < maxIter; it++) {
    const { r, J } = evaluate(sys, q);
    const res = r.reduce((m, v) => Math.max(m, Math.abs(v)), 0);
    if (res < CORRECT_TOL) return { q, ok: true, residual: res };
    const m = r.length;
    const JJt = Array.from({ length: m }, (_, a) =>
      Array.from({ length: m }, (_, b) => J[a].reduce((s, v, k) => s + v * J[b][k], 0) + (a === b ? 1e-10 : 0)),
    );
    const y = solve(JJt, r);
    if (!y) return { q, ok: false, residual: res };
    q = q.map((v, k) => v - J.reduce((s, row, a) => s + row[k] * y[a], 0));
  }
  const { r } = evaluate(sys, q);
  const residual = r.reduce((m, v) => Math.max(m, Math.abs(v)), 0);
  return { q, ok: residual < 1e-8, residual };
}

/**
 * One tangent step of size h along the current mechanism motion followed by Newton
 * correction. The direction is the previous direction projected onto the current null
 * space (keeps the sense of motion continuous); without one, mode `modeIndex` is used.
 * Returns { ok, q, dir }; ok=false when there is no motion or the step cannot be corrected.
 */
export function motionStep(sys, q, prevDir, h, modeIndex = 0) {
  const { modes } = analyze(sys, q);
  if (!modes.length) return { ok: false, q, dir: prevDir };
  let d = null;
  if (prevDir) {
    d = new Array(prevDir.length).fill(0);
    for (const m of modes) {
      const k = m.reduce((s, v, i) => s + v * prevDir[i], 0);
      m.forEach((v, i) => (d[i] += k * v));
    }
    d = normalizeMode(sys, q, d);
  }
  if (!d) d = normalizeMode(sys, q, modes[Math.min(modeIndex, modes.length - 1)]);
  if (!d) return { ok: false, q, dir: prevDir };
  let step = h;
  for (let attempt = 0; attempt < 5; attempt++, step /= 2) {
    const trial = correct(sys, q.map((v, i) => v + step * d[i]));
    if (trial.ok) return { ok: true, q: trial.q, dir: d };
  }
  return { ok: false, q, dir: d };
}

/**
 * Drag: move the body-fixed point `pRef` of body i towards the world point `target`
 * as far as the constraints allow. Each iteration picks the combination of mechanism
 * modes whose velocity at the grab point best matches the remaining offset (least
 * squares), then corrects back onto the constraint manifold.
 */
export function dragTo(sys, q0, i, pRef, target, maxSteps = 8, maxStepLen = 0.25) {
  let q = q0;
  for (let s = 0; s < maxSteps; s++) {
    const P = worldPoint(q, i, pRef);
    let delta = sub(target, P);
    const l = len(delta);
    if (l < 1e-4) break;
    if (l > maxStepLen) delta = mul(delta, maxStepLen / l);
    const { modes } = analyze(sys, q);
    const K = modes.length;
    if (!K) break;
    const A = modes.map((m) => pointVelocity(q, m, i, P)); // A[k] = grab-point velocity of mode k
    const M = Array.from({ length: K }, (_, a) =>
      Array.from({ length: K }, (_, b) => A[a].x * A[b].x + A[a].y * A[b].y + (a === b ? 1e-9 : 0)),
    );
    const rhs = A.map((v) => v.x * delta.x + v.y * delta.y);
    const alpha = solve(M, rhs);
    if (!alpha) break;
    const step = q.map((v, k) => v + modes.reduce((sum, m, a) => sum + alpha[a] * m[k], 0));
    const c = correct(sys, step);
    if (!c.ok) break;
    const moved = dist(worldPoint(c.q, i, pRef), P);
    q = c.q;
    if (moved < 1e-6) break;
  }
  return q;
}
