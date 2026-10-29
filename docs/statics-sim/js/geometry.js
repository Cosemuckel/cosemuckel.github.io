// Pure 2D geometry helpers (no DOM). World coordinates are y-up.

export const EPS = 1e-9;

export const add = (a, b) => ({ x: a.x + b.x, y: a.y + b.y });
export const sub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y });
export const mul = (a, k) => ({ x: a.x * k, y: a.y * k });
export const dot = (a, b) => a.x * b.x + a.y * b.y;
export const cross = (a, b) => a.x * b.y - a.y * b.x;
export const len = (a) => Math.hypot(a.x, a.y);
export const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
/** Rotate 90 degrees counter-clockwise. */
export const perp = (a) => ({ x: -a.y, y: a.x });
export const unit = (a) => {
  const l = len(a);
  return l < EPS ? { x: 0, y: 0 } : { x: a.x / l, y: a.y / l };
};
export const fromAngleDeg = (deg) => {
  const r = (deg * Math.PI) / 180;
  return { x: Math.cos(r), y: Math.sin(r) };
};

/**
 * Intersection of two infinite lines, each given as { p: point, d: direction }.
 *
 * Returns one of
 *   { kind: 'point', point, t1, t2 }   p1 + t1*d1 === p2 + t2*d2
 *   { kind: 'parallel', direction }    distinct parallel lines: they meet at
 *                                      infinity in the (unit) direction of d1
 *   { kind: 'coincident' }             same line: every point is common
 */
export function intersectLines(l1, l2, eps = EPS) {
  const d1 = unit(l1.d);
  const d2 = unit(l2.d);
  const denom = cross(d1, d2);
  const w = sub(l2.p, l1.p);
  if (Math.abs(denom) < eps) {
    // Parallel: coincident iff the offset between anchor points is also parallel to d1.
    if (Math.abs(cross(w, d1)) < 1e-7) return { kind: 'coincident' };
    return { kind: 'parallel', direction: d1 };
  }
  const t1 = cross(w, d2) / denom;
  const t2 = cross(w, d1) / denom;
  return { kind: 'point', point: add(l1.p, mul(d1, t1)), t1, t2 };
}

export function closestPointOnSegment(p, a, b) {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  if (l2 < EPS) return { ...a };
  const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2));
  return add(a, mul(ab, t));
}

export function distPointSegment(p, a, b) {
  return dist(p, closestPointOnSegment(p, a, b));
}

export function snapValue(v, step) {
  return Number((Math.round(v / step) * step).toFixed(9));
}

/** Smallest "1-2-5" grid step whose on-screen size is at least minPx. */
export function gridStep(scale, minPx = 30) {
  const steps = [];
  for (let e = -3; e <= 4; e++) for (const m of [1, 2, 5]) steps.push(m * 10 ** e);
  return steps.find((s) => s * scale >= minPx) ?? steps[steps.length - 1];
}

/**
 * Point where a ray from `origin` along `dir` leaves the rectangle
 * {x0,y0,x1,y1} (screen space). An origin outside the rectangle is clamped in first.
 */
export function clipRayToRect(origin, dir, rect) {
  const o = {
    x: Math.min(rect.x1, Math.max(rect.x0, origin.x)),
    y: Math.min(rect.y1, Math.max(rect.y0, origin.y)),
  };
  let tMax = Infinity;
  if (Math.abs(dir.x) > EPS) tMax = Math.min(tMax, ((dir.x > 0 ? rect.x1 : rect.x0) - o.x) / dir.x);
  if (Math.abs(dir.y) > EPS) tMax = Math.min(tMax, ((dir.y > 0 ? rect.y1 : rect.y0) - o.y) / dir.y);
  if (!Number.isFinite(tMax)) return o;
  return add(o, mul(dir, Math.max(0, tMax)));
}
