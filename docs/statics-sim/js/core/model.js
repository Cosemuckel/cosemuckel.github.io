// @ts-check
// Scene model (pure data + helpers, no DOM).
//
// Bodies are rigid and stored in their reference configuration:
//   { id, kind: 'rod', pts: [{x,y}, ...], segs?: [[i,j], ...] }
//     pts = nodes; segs = straight segments between node indices. Without `segs` the
//     nodes form an open chain (a plain rod has 2 points). Segments may share nodes
//     (frames, T/L/Z shapes, branches, loops) and are all one rigid body.
// Supports are located by a world point `p`; which bodies they act on is derived from
// geometry (see bodiesAtPoint), so moving things around never leaves stale references:
//   pin    : topmost body at p is pinned to the ground (optionally clamped = no rotation)
//   roller : topmost body at p may only slide along direction `angleDeg` (ground rail);
//            `locked` removes the sliding freedom (acts like a pin)
//   joint  : all bodies at p are hinged together (needs at least two)

import {
  distPointSegment, closestPointOnSegment, dist,
} from './geometry.js';

/** How close (world units) a support point must be to a body to attach to it. */
export const ATTACH_TOL = 0.25;
const NODE_TOL = 1e-6; // points closer than this are the same node

export function createScene() {
  return { bodies: [], supports: [], nextId: 1 };
}

export function addRod(scene, a, b) {
  const body = { id: scene.nextId++, kind: 'rod', pts: [{ ...a }, { ...b }] };
  scene.bodies.push(body);
  return body;
}

/** Index pairs of the drawn segments of a body (segs, or the open chain through its points). */
export function bodySegments(kind, pts, segs) {
  if (segs) return segs;
  return pts.slice(1).map((_, i) => [i, i + 1]);
}

/**
 * Appends the segments of the point chain `chain` to a rod body, reusing nodes that
 * coincide with existing ones (so a chain can start on, end on or close onto the body).
 * Zero-length and duplicate segments are skipped. Returns the number of segments added.
 */
function appendChain(body, chain) {
  const segs = bodySegments(body.kind, body.pts, body.segs).map((s) => [...s]);
  const nodeIndex = (p) => {
    let k = body.pts.findIndex((q) => dist(q, p) < NODE_TOL);
    if (k < 0) k = body.pts.push({ ...p }) - 1;
    return k;
  };
  const before = segs.length;
  let prev = nodeIndex(chain[0]);
  for (const p of chain.slice(1)) {
    const k = nodeIndex(p);
    if (k !== prev && !segs.some(([i, j]) => (i === prev && j === k) || (i === k && j === prev))) segs.push([prev, k]);
    prev = k;
  }
  body.segs = segs;
  return segs.length - before;
}

/** One rigid body from a chain of connected segments (2 points = a plain rod). Null if degenerate. */
export function addPolyline(scene, chain) {
  const body = { id: scene.nextId, kind: 'rod', pts: [] };
  if (!appendChain(body, chain)) return null;
  if (body.segs.length === 1) delete body.segs; // plain rod: nodes 0-1
  scene.nextId++;
  scene.bodies.push(body);
  return body;
}

/** Adds the chain's segments to an existing rod body. Returns the number of segments added. */
export function extendBody(scene, id, chain) {
  const body = scene.bodies.find((b) => b.id === id && b.kind === 'rod');
  return body ? appendChain(body, chain) : 0;
}

/** Merges rod `otherId` into rod `keepId` (one rigid body); joints between them become invalid and are dropped. */
export function mergeBodies(scene, keepId, otherId) {
  const keep = scene.bodies.find((b) => b.id === keepId && b.kind === 'rod');
  const other = scene.bodies.find((b) => b.id === otherId && b.kind === 'rod');
  if (!keep || !other || keep === other) return false;
  for (const [i, j] of bodySegments(other.kind, other.pts, other.segs)) appendChain(keep, [other.pts[i], other.pts[j]]);
  scene.bodies = scene.bodies.filter((b) => b !== other);
  scene.supports = scene.supports.filter((s) => supportIsValid(scene, s));
  return true;
}

/** Point on the body's segments closest to p. */
export function nearestPointOnBody(body, p) {
  let best = null;
  let bestD = Infinity;
  for (const [i, j] of bodySegments(body.kind, body.pts, body.segs)) {
    const c = closestPointOnSegment(p, body.pts[i], body.pts[j]);
    const d = dist(c, p);
    if (d < bestD) { bestD = d; best = c; }
  }
  return best;
}

export function addSupport(scene, type, p, opts = {}) {
  const support = {
    id: scene.nextId++,
    type,
    p: { ...p },
    angleDeg: opts.angleDeg ?? 0, // roller: direction of the permitted sliding movement
    locked: !!opts.locked, // roller: slide locked
    clamped: !!opts.clamped, // pin: also blocks rotation (fixed-in-place)
    showBeam: opts.showBeam ?? true,
  };
  scene.supports.push(support);
  return support;
}

/** True when point p lies within tol of one of the body's segments. */
export function pointHitsShape(kind, pts, p, tol, segs) {
  return bodySegments(kind, pts, segs).some(([i, j]) => distPointSegment(p, pts[i], pts[j]) <= tol);
}

/** Bodies containing p, in scene order (last = drawn on top). */
export function bodiesAtPoint(scene, p, tol = ATTACH_TOL) {
  return scene.bodies.filter((b) => pointHitsShape(b.kind, b.pts, p, tol, b.segs));
}

export function supportIsValid(scene, s) {
  const n = bodiesAtPoint(scene, s.p).length;
  return s.type === 'joint' ? n >= 2 : n >= 1;
}

export function removeSupport(scene, id) {
  scene.supports = scene.supports.filter((s) => s.id !== id);
}

/** Removes a body and every support that no longer has a body to act on. */
export function removeBody(scene, id) {
  scene.bodies = scene.bodies.filter((b) => b.id !== id);
  scene.supports = scene.supports.filter((s) => supportIsValid(scene, s));
}

export function cloneScene(scene) {
  return JSON.parse(JSON.stringify(scene));
}

// ---------------------------------------------------------------- examples

/** Two bars A-B-C, pin at A, hinge at B, horizontal roller at C: 1 degree of freedom. */
function twoBarMechanism() {
  const s = createScene();
  addRod(s, { x: -4, y: 0 }, { x: -1, y: 3 });
  addRod(s, { x: -1, y: 3 }, { x: 3, y: 0 });
  addSupport(s, 'pin', { x: -4, y: 0 });
  addSupport(s, 'joint', { x: -1, y: 3 });
  addSupport(s, 'roller', { x: 3, y: 0 }, { angleDeg: 0 });
  return s;
}

/** Three-hinged arch: two pins and a crown hinge, rigid and statically determinate. */
function threeHingedArch() {
  const s = createScene();
  addRod(s, { x: -4, y: 0 }, { x: 0, y: 4 });
  addRod(s, { x: 0, y: 4 }, { x: 4, y: 0 });
  addSupport(s, 'pin', { x: -4, y: 0 });
  addSupport(s, 'joint', { x: 0, y: 4 });
  addSupport(s, 'pin', { x: 4, y: 0 });
  return s;
}

/** One L-shaped frame (column + beam, a single rigid body) on a pin and a horizontal roller: rigid. */
function lFrameOnPinAndRoller() {
  const s = createScene();
  addPolyline(s, [{ x: -3, y: 0 }, { x: -3, y: 4 }, { x: 3, y: 4 }]);
  addSupport(s, 'pin', { x: -3, y: 0 });
  addSupport(s, 'roller', { x: 3, y: 4 }, { angleDeg: 0 });
  return s;
}

/** Slider-crank: crank (length 2) on a pin, coupler to a roller on a horizontal rail: 1 degree of freedom. */
function sliderCrank() {
  const s = createScene();
  const x = 4.5;
  addRod(s, { x: 0, y: 0 }, { x: 0, y: 2 });
  addRod(s, { x: 0, y: 2 }, { x, y: 0 });
  addSupport(s, 'pin', { x: 0, y: 0 });
  addSupport(s, 'joint', { x: 0, y: 2 });
  addSupport(s, 'roller', { x, y: 0 }, { angleDeg: 0 });
  return s;
}

/** Three bars pin - hinge - hinge - roller (horizontal): 2 degrees of freedom. */
function threeBarChain() {
  const s = createScene();
  addRod(s, { x: -4, y: 0 }, { x: -2, y: 3 });
  addRod(s, { x: -2, y: 3 }, { x: 2, y: 3 });
  addRod(s, { x: 2, y: 3 }, { x: 4, y: 0 });
  addSupport(s, 'pin', { x: -4, y: 0 });
  addSupport(s, 'joint', { x: -2, y: 3 });
  addSupport(s, 'joint', { x: 2, y: 3 });
  addSupport(s, 'roller', { x: 4, y: 0 }, { angleDeg: 0 });
  return s;
}

export const EXAMPLES = {
  mechanism: { label: 'Two-bar mechanism (1 DOF)', build: twoBarMechanism },
  chain: { label: 'Three-bar chain (2 DOF)', build: threeBarChain },
  slidercrank: { label: 'Slider-crank (1 DOF)', build: sliderCrank },
  arch: { label: 'Three-hinged arch (rigid)', build: threeHingedArch },
  frame: { label: 'L-shaped frame on pin + roller (rigid)', build: lFrameOnPinAndRoller },
};
