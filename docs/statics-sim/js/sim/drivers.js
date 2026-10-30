// @ts-check
// Driver coordinates of a mechanism (pure maths, no DOM).
//
// A mechanism with k degrees of freedom is steered by k scalar "drivers": functions g(q) of the pose whose
// gradient rows, restricted to the null space of the constraint Jacobian, are linearly independent. Holding k - 1
// of them leaves a one-degree-of-freedom curve along which the remaining driver is the parameter.
//
// A Driver is { id, kind, label, value(q), gradient(q), order, body?, index?, rowScale? }:
//   rotation  id `r<body>`      the angle of a body (q coordinate, `index`)
//   x / y     id `x<body>`      displacement of the centre of a body's nodes (mean of its reference nodes) along x / y;
//             id `y<body>`      not a q coordinate, so no `index` (see centreDriver)
//   slide     id `s<support>`   travel of a roller's attached point along its rail (projection of the world point
//                               onto the rail direction); `<support>` is the scene support id
//   relAngle  id `a<A>:<B>`     q[rot A] - q[rot B] for two bodies hinged by a joint
// `rowScale` (x / y / slide): the divisor of the driver's row in the independence tests instead of the length of its q
// gradient. That length contains the lever arm of the body about the world origin (rotation enters through it), so it
// would make the verdicts depend on where the scene sits in the world; the rate of the driver itself does not.
// Bodies are numbered from 0 in ids (labels count from 1). Ids are stable while the scene keeps its bodies and
// supports, so a selection can be stored and restored.
//
// CANONICAL ORDER (defined here and nowhere else; it orders the candidate lists, the default selection and so the
// numbering of the modes). It is a total order that depends on the structure of the mechanism only, never on the pose:
//   1. rotation of a body that is pinned to the ground (a pin or a locked roller), by body index
//   2. rotation of any other body, by body index
//   3. relative angle of two hinged bodies, by body pair (A, then B)
//   4. roller slide, by support order
//   5. x translation, by body index
//   6. y translation, by body index
// Ties (none occur in a valid scene) are broken by the id. New bodies and supports get the highest index, so they come
// last within their class.
//
// DEFAULT SELECTION: walk the canonical order and take every driver that keeps the chosen ones independent, until k are
// chosen. SLOTS: a selection is a list in slot order (slot i is "Mode i + 1"). The default list is in canonical order; a
// driver the user replaces keeps its slot, so a stored list is read as slot order and is never re-sorted.
//
// INDEPENDENCE is judged without regard to the pose: rows of the driver gradients on the null space are compared at the
// pose itself and, when that fails, at a few fixed nearby poses of the same mechanism (see perturbedFrames), so a driver
// whose gradient vanishes exactly at a dead point is not rejected there.
// The tests run in the mechanism's own frame (the scene translated so that the centroid of its reference nodes is the
// world origin; see createJudge), so their verdicts do not depend on how far the scene is from the world origin.

import { analyze, correct, worldPoint } from '../core/kinematics.js';
import { inverse } from '../core/linalg.js';

/** Independence thresholds tried in turn when picking drivers (relative size of a row left after the earlier rows). */
export const DRIVER_TOLS = [1e-3, 1e-6];
/** Threshold for validating a given selection and listing candidates. */
export const CANDIDATE_TOL = DRIVER_TOLS[DRIVER_TOLS.length - 1];

const dotv = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

const CLASS = { pinnedRotation: 0, rotation: 1, relAngle: 2, slide: 3, x: 4, y: 5 };

function coordinateDriver(sys, id, kind, body, index, label, order) {
  const size = 3 * sys.n;
  const gradient = () => {
    const g = new Array(size).fill(0);
    g[index] = 1;
    return g;
  };
  return { id, kind, label, body, index, order, value: (q) => q[index], gradient };
}

/** Mean of the body's nodes in the reference configuration: a point of the body that does not depend on the world origin. */
function bodyCentre(sys, body) {
  const pts = sys.bodies[body].pts;
  return { x: pts.reduce((s, p) => s + p.x, 0) / pts.length, y: pts.reduce((s, p) => s + p.y, 0) / pts.length };
}

/**
 * Displacement of the body's centre (see bodyCentre) along x (`axis` 0) or y (`axis` 1) from the reference configuration.
 * q holds the body's translation of the world-origin image, so the coordinate q[3b] would be the motion of a point that
 * coincides with a pin whenever the mechanism happens to be drawn there; the centre is a property of the body itself,
 * so what is constant (and so not offered) depends on the mechanism only, not on where it sits in the world.
 */
function centreDriver(sys, id, kind, body, axis, label, order) {
  const size = 3 * sys.n;
  const c = bodyCentre(sys, body);
  const j = 3 * body;
  const key = axis ? 'y' : 'x';
  return {
    id,
    kind,
    label,
    body,
    order,
    rowScale: 1,
    value: (q) => worldPoint(q, body, c)[key] - c[key],
    gradient: (q) => {
      const g = new Array(size).fill(0);
      g[j + axis] = 1;
      const th = q[j + 2];
      g[j + 2] = axis ? Math.cos(th) * c.x - Math.sin(th) * c.y : -Math.sin(th) * c.x - Math.cos(th) * c.y;
      return g;
    },
  };
}

function slideDriver(sys, c, position) {
  const size = 3 * sys.n;
  const rail = { x: c.n.y, y: -c.n.x }; // n = perp(rail direction)
  const j = 3 * c.body;
  return {
    id: `s${c.support}`,
    kind: 'slide',
    label: `slide of the roller on body ${c.body + 1}`,
    body: c.body,
    rowScale: 1,
    order: [CLASS.slide, position, 0],
    value: (q) => {
      const P = worldPoint(q, c.body, c.p);
      return rail.x * P.x + rail.y * P.y;
    },
    gradient: (q) => {
      const g = new Array(size).fill(0);
      const th = q[j + 2];
      const dP = { x: -Math.sin(th) * c.p.x - Math.cos(th) * c.p.y, y: Math.cos(th) * c.p.x - Math.sin(th) * c.p.y };
      g[j] = rail.x;
      g[j + 1] = rail.y;
      g[j + 2] = rail.x * dP.x + rail.y * dP.y;
      return g;
    },
  };
}

function relativeAngleDriver(sys, a, b) {
  const size = 3 * sys.n;
  return {
    id: `a${a}:${b}`,
    kind: 'relAngle',
    label: `angle between bodies ${a + 1} and ${b + 1}`,
    body: a,
    other: b,
    order: [CLASS.relAngle, a, b],
    value: (q) => q[3 * a + 2] - q[3 * b + 2],
    gradient: () => {
      const g = new Array(size).fill(0);
      g[3 * a + 2] = 1;
      g[3 * b + 2] = -1;
      return g;
    },
  };
}

/** The canonical order (see the header): negative when driver `a` comes first. Total on distinct ids. */
export function compareDrivers(a, b) {
  for (let i = 0; i < a.order.length; i++) if (a.order[i] !== b.order[i]) return a.order[i] - b.order[i];
  return a.id < b.id ? -1 : a.id > b.id ? 1 : 0;
}

/** `drivers` sorted in the canonical order (a new array; the order of the input never matters). */
export const canonicalOrder = (drivers) => drivers.slice().sort(compareDrivers);

/**
 * Every driver candidate of `sys` in the canonical order (see the header). Which of them can be used together is
 * decided by defaultSelection / validateDriverSelection / candidatesPerMode.
 */
export function listDrivers(sys) {
  const out = [];
  const pinned = new Set(sys.constraints.filter((c) => c.kind === 'pin').map((c) => c.body));
  for (let b = 0; b < sys.n; b++) {
    const cls = pinned.has(b) ? CLASS.pinnedRotation : CLASS.rotation;
    out.push(coordinateDriver(sys, `r${b}`, 'rotation', b, 3 * b + 2, `rotation of body ${b + 1}`, [cls, b, 0]));
    out.push(centreDriver(sys, `x${b}`, 'x', b, 0, `translation of body ${b + 1} in x`, [CLASS.x, b, 0]));
    out.push(centreDriver(sys, `y${b}`, 'y', b, 1, `translation of body ${b + 1} in y`, [CLASS.y, b, 0]));
  }
  let position = 0;
  for (const c of sys.constraints) if (c.kind === 'slide') out.push(slideDriver(sys, c, position++));
  const seen = new Set();
  for (const c of sys.constraints) {
    if (c.kind !== 'joint' || c.a === c.b) continue;
    const key = `${c.a}:${c.b}`;
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(relativeAngleDriver(sys, c.a, c.b));
  }
  return canonicalOrder(out);
}

/** `row` made orthogonal to the orthonormal list `ortho` and normalised; null when less than `tol` of it is left. */
function orthoResidual(row, ortho, tol) {
  const w = row.slice();
  for (const u of ortho) {
    const c = dotv(w, u);
    u.forEach((v, i) => (w[i] -= c * v));
  }
  const l = Math.sqrt(dotv(w, w));
  return l <= tol ? null : w.map((v) => v / l);
}

/** True when the rows are linearly independent: each keeps more than `tol` after removing the rows before it. */
function independent(rows, tol) {
  const ortho = [];
  for (const row of rows) {
    const unit = orthoResidual(row, ortho, tol);
    if (!unit) return false;
    ortho.push(unit);
  }
  return true;
}

/** Distances (in q) of the nearby poses at which independence is also judged. */
const FRAME_STEPS = [0.03, -0.03, 0.1, -0.1, 0.3, -0.3];
/**
 * Threshold in the nearby frames. At a singular pose (a dead point, a stroke limit) two rows that are independent
 * almost everywhere coincide up to rounding (~1e-9), and the row left over grows only linearly with the distance from
 * it (about 1e-3 per unit in a dyad chain); so a nearby frame cannot be held to the threshold of the pose itself. A
 * pair that is dependent everywhere stays at rounding level (<= 1e-12) in every frame.
 */
const FRAME_TOL = 1e-5;
const GOLDEN = 0.6180339887498949;

/**
 * Nearby poses of the same mechanism, { q, modes }, with as many modes as `modes`. The direction is the projection on
 * the null space of a fixed generic vector (the projection does not depend on which basis `modes` is); the poses are the
 * Newton corrections of the steps FRAME_STEPS along it. Poses that cannot be reached or whose mobility differs are left out.
 */
function perturbedFrames(sys, q, modes) {
  const w = q.map((_, j) => (((j + 1) * GOLDEN) % 1) + 0.25);
  const dir = new Array(q.length).fill(0);
  for (const m of modes) {
    const c = dotv(m, w);
    m.forEach((v, i) => (dir[i] += c * v));
  }
  const l = Math.sqrt(dotv(dir, dir));
  if (l < 1e-9) return [];
  const frames = [];
  for (const h of FRAME_STEPS) {
    const moved = correct(sys, q.map((v, i) => v + (h * dir[i]) / l));
    if (!moved.ok) continue;
    const found = analyze(sys, moved.q);
    if (found.nullity === modes.length) frames.push({ q: moved.q, modes: found.modes });
  }
  return frames;
}

const shiftedSystems = new WeakMap(); // sys -> { shift, sys, drivers by id } (null when the system cannot be shifted)

/**
 * Copy of `sys` translated so that the mean of all reference nodes of all bodies is the world origin, with its drivers.
 * Null when there is nothing to shift or the system holds constraints given in q coordinates (fix / driver rows).
 */
function shiftedSystem(sys) {
  if (shiftedSystems.has(sys)) return shiftedSystems.get(sys);
  let found = null;
  const pts = sys.bodies.flatMap((b) => b.pts);
  if (pts.length && sys.constraints.every((c) => c.kind !== 'fix' && c.kind !== 'driver')) {
    const shift = { x: pts.reduce((t, p) => t + p.x, 0) / pts.length, y: pts.reduce((t, p) => t + p.y, 0) / pts.length };
    const move = (p) => ({ x: p.x - shift.x, y: p.y - shift.y });
    const moved = {
      ...sys,
      bodies: sys.bodies.map((b) => ({ ...b, pts: b.pts.map(move) })),
      constraints: sys.constraints.map((c) => (c.p ? { ...c, p: move(c.p) } : c)),
    };
    found = { shift, sys: moved, drivers: new Map(listDrivers(moved).map((d) => [d.id, d])) };
  }
  shiftedSystems.set(sys, found);
  return found;
}

/** Pose q of `sys` as the pose of the shifted system (same rotations, translations t + (R - I) shift). */
function shiftedPose(q, shift) {
  return q.map((v, j) => {
    if (j % 3 === 2) return v;
    const th = q[j - (j % 3) + 2];
    const c = Math.cos(th) - 1;
    const s = Math.sin(th);
    return v + (j % 3 === 0 ? c * shift.x - s * shift.y : s * shift.x + c * shift.y);
  });
}

/**
 * Pose-robust independence tests for the drivers of `sys` at pose `q` (`modes` = orthonormal null-space basis there).
 * A set of drivers counts as independent when its rows are independent at q or, failing that, in one of the nearby
 * frames (built on first need). A row is the gradient restricted to the null space divided by the gradient's length
 * (by `rowScale` where a driver has one), so a threshold is the sine of the angle between the gradient and the space it
 * must not lie in.
 *
 * The tests run in the mechanism's own frame: a copy of the system translated so that the centroid of its reference
 * nodes is the world origin (see shiftedSystem). q holds rotations about the world origin, so in the scene's own frame
 * the lever arm of every body grows with the distance of the scene from the origin and the orthonormal null-space basis
 * turns into almost pure translations; the rank tests would then depend on where the scene sits. The frame is only used
 * here: drivers, poses and modes handed in and out stay in the scene's frame. When the shifted pose has a different
 * mobility (a borderline numerical case) the scene's own frame is used.
 */
function createJudge(sys, q, modes) {
  let local = { sys, q, modes, driverOf: (d) => d };
  const shifted = shiftedSystem(sys);
  if (shifted) {
    const qLocal = shiftedPose(q, shifted.shift);
    const found = analyze(shifted.sys, qLocal);
    if (found.nullity === modes.length) local = { sys: shifted.sys, q: qLocal, modes: found.modes, driverOf: (d) => shifted.drivers.get(d.id) || d };
  }
  let nearby = null;
  const rowCache = new Map(); // driver -> rows by frame index
  const rowIn = (index, frame, driver) => {
    let rows = rowCache.get(driver);
    if (!rows) rowCache.set(driver, (rows = []));
    if (!rows[index]) {
      const own = local.driverOf(driver);
      const g = own.gradient(frame.q);
      const norm = own.rowScale || Math.sqrt(dotv(g, g)) || 1;
      rows[index] = frame.modes.map((m) => dotv(g, m) / norm);
    }
    return rows[index];
  };
  /** The pose itself first; the nearby frames are only built when a caller asks for the next one. */
  function* frames() {
    yield { index: 0, frame: { q: local.q, modes: local.modes } };
    if (!nearby) nearby = perturbedFrames(local.sys, local.q, local.modes);
    for (let i = 0; i < nearby.length; i++) yield { index: i + 1, frame: nearby[i] };
  }
  return {
    rowIn,
    frames,
    independentSet(drivers, tol) {
      for (const { index, frame } of frames()) if (independent(drivers.map((d) => rowIn(index, frame, d)), index ? Math.min(tol, FRAME_TOL) : tol)) return true;
      return false;
    },
  };
}

/**
 * The default selection of `modes.length` drivers: the canonical order is walked and a driver is taken when the set
 * stays independent (see the header). The result is in canonical order. `drivers` defaults to listDrivers(sys); its
 * order does not matter. Returns null when no independent set exists.
 */
export function defaultSelection(sys, q, modes, drivers = listDrivers(sys)) {
  const k = modes.length;
  const ordered = canonicalOrder(drivers);
  const judge = createJudge(sys, q, modes);
  for (const tol of DRIVER_TOLS) {
    const chosen = [];
    for (const d of ordered) {
      if (judge.independentSet([...chosen, d], tol)) chosen.push(d);
      if (chosen.length === k) return chosen;
    }
  }
  return null;
}

/**
 * Checks a stored selection `ids` (array of driver ids, one per mode, in slot order) at pose q.
 * Returns { ok, reason, drivers }: `drivers` is the resolved list when ok; `reason` says what is wrong otherwise
 * (not an array, duplicate or unknown id, wrong count, dependent drivers).
 */
export function validateDriverSelection(sys, q, ids, modes = analyze(sys, q).modes) {
  const bad = (reason) => ({ ok: false, reason, drivers: null });
  if (!Array.isArray(ids)) return bad('the driver selection must be a list of ids');
  const k = modes.length;
  if (ids.length !== k) return bad(`${k} driver(s) needed for ${k} degree(s) of freedom, ${ids.length} given`);
  const all = new Map(listDrivers(sys).map((d) => [d.id, d]));
  const chosen = [];
  for (const id of ids) {
    const d = all.get(id);
    if (!d) return bad(`unknown driver '${String(id)}'`);
    if (chosen.includes(d)) return bad(`driver '${id}' is selected twice`);
    chosen.push(d);
  }
  if (!createJudge(sys, q, modes).independentSet(chosen, CANDIDATE_TOL)) return bad(`drivers ${ids.join(', ')} are not independent at this pose`);
  return { ok: true, reason: null, drivers: chosen };
}

/**
 * Valid drivers for each slot of `selection` (array of drivers in slot order, one per mode), in the canonical order:
 * the candidates that keep the other slots' drivers independent when they take this slot (its own driver included).
 * Judged like the default selection (a candidate counts when it qualifies at the pose or in a nearby frame), so the
 * lists do not change with the pose. For one degree of freedom a candidate is valid when it changes along the motion.
 */
export function candidatesPerMode(sys, q, modes, selection, drivers = listDrivers(sys)) {
  const k = modes.length;
  const ordered = canonicalOrder(drivers);
  const judge = createJudge(sys, q, modes);
  const valid = Array.from({ length: k }, () => new Set());
  for (const { index, frame } of judge.frames()) {
    const tol = index ? Math.min(CANDIDATE_TOL, FRAME_TOL) : CANDIDATE_TOL;
    for (let i = 0; i < k; i++) {
      // Orthonormal basis of the other slots' rows in this frame; the frame says nothing if they are dependent there.
      const ortho = [];
      let usable = true;
      for (let j = 0; j < k && usable; j++) {
        if (j === i || !selection[j]) continue;
        const unit = orthoResidual(judge.rowIn(index, frame, selection[j]), ortho, tol);
        if (unit) ortho.push(unit);
        else usable = false;
      }
      if (!usable) continue;
      for (const d of ordered) {
        if (!valid[i].has(d) && orthoResidual(judge.rowIn(index, frame, d), ortho, tol)) valid[i].add(d);
      }
    }
  }
  return valid.map((set, i) => ordered.filter((d) => set.has(d) || d === selection[i]));
}

/**
 * The motion that slot i stands for at pose q: the null-space velocity (length 3n) along which driver i changes at unit
 * rate while the gradients of the other slots' drivers stay at zero (column i of the inverse of the selection matrix, in
 * the basis `modes`). It is the instantaneous direction of the mode's curve in trajectory.js and does not depend on the
 * basis `modes` happens to be. Null when the selection is singular at q.
 */
export function slotMotions(q, modes, selection) {
  const rows = selection.map((d) => {
    const g = d.gradient(q);
    return modes.map((m) => dotv(g, m));
  });
  const inv = inverse(rows);
  if (!inv) return null;
  return selection.map((_, i) => {
    const v = new Array(q.length).fill(0);
    modes.forEach((m, a) => m.forEach((x, c) => (v[c] += inv[a][i] * x)));
    return v;
  });
}
