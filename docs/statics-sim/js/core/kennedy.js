// @ts-check
// Relative poles and Kennedy's three-pole theorem (pure maths, no DOM).
//
// Bodies are numbered 1..n in scene order; the ground is body 0. The relative pole (i, j) is the
// point where bodies i and j have the same velocity (the absolute pole of body k is (0, k)). It may
// lie at infinity (pure relative translation), described by a direction.
//
// Kennedy: for any three bodies i, j, m the poles (i,j), (j,m), (i,m) lie on one line. So an unknown
// pole (i,j) lies on the line through (i,m) and (j,m) for every m whose two poles are known, and
// on the roller beam (a line through the support) for (0,k). Two different lines fix the pole.

import { intersectLines, dist, sub, unit } from './geometry.js';

const SAME_POINT = 1e-9;

/** Canonical key of the pole between bodies i and j (order does not matter). */
export const poleKey = (i, j) => (i < j ? `${i}-${j}` : `${j}-${i}`);

/** Display notation, e.g. "(0,2)"; a pole at infinity adds the infinity sign: "(0,2)∞". */
export const poleName = (i, j, atInfinity = false) => `(${Math.min(i, j)},${Math.max(i, j)})${atInfinity ? '∞' : ''}`;

/** Label of the poles sharing one location, e.g. "(1,2)" or "(1,2)=(1,3)=(2,3)" for a joint of three bodies. */
export const pairsLabel = (pairs) => pairs.map(([i, j]) => poleName(i, j)).join('=');

/** Label of a pole returned by derivePoles(), e.g. "(0,2)" or "(0,2)∞". */
export const derivedLabel = (pole) => poleName(...pole.pair, pole.kind === 'infinity');

/** The line a pole lies on given two other poles (the two Kennedy partners), or null if they fix no line. */
function lineThrough(a, b) {
  if (a.kind === 'point' && b.kind === 'point') {
    if (dist(a.point, b.point) < SAME_POINT) return null;
    return { p: a.point, d: unit(sub(b.point, a.point)) };
  }
  if (a.kind === 'point' && b.kind === 'infinity') return { p: a.point, d: b.direction };
  if (a.kind === 'infinity' && b.kind === 'point') return { p: b.point, d: a.direction };
  return null; // two points at infinity span only the line at infinity
}

/**
 * Derives poles from the support poles by repeated use of Kennedy's theorem.
 *
 * `items` are poleItems() results: point items carry `pairs` (the relative poles located at
 * `point`), line items (roller beams) carry the single pair whose pole lies on the beam.
 * `bodyCount` is the number of bodies (bodies 1..n, ground 0).
 *
 * Returns a Map poleKey -> pole:
 *   { key, pair: [i, j], kind: 'point', point } | { key, pair, kind: 'infinity', direction },
 *   origin: 'support' (located directly by a pin / joint / locked roller)
 *         | 'derived' (found as the intersection of the two lines in `via`)
 * `via` entries are { p, d, source } where source is { type: 'beam', id } (a roller beam) or
 * { type: 'kennedy', through: [keyA, keyB], body: m } (the line through poles keyA and keyB).
 * Poles that cannot be determined (fewer than two independent lines) are absent.
 */
export function derivePoles(items, bodyCount) {
  const size = bodyCount + 1; // bodies 1..n, ground 0
  // Poles and beams are looked up by the integer slot of the (sorted) body pair; the string key only labels a pole.
  const slot = (i, j) => (i < j ? i * size + j : j * size + i);
  const grid = new Array(size * size).fill(null);
  const beams = new Map(); // slot -> beam lines
  const poles = new Map();
  const add = (pole) => {
    grid[slot(...pole.pair)] = pole;
    poles.set(pole.key, pole);
  };
  for (const it of items) {
    if (it.kind === 'line') {
      const [i, j] = it.pairs[0];
      const at = slot(i, j);
      if (!beams.has(at)) beams.set(at, []);
      beams.get(at).push({ p: it.point, d: it.dir, source: { type: 'beam', id: it.id } });
    } else {
      for (const [i, j] of it.pairs) {
        if (!grid[slot(i, j)]) add({ key: poleKey(i, j), pair: [i, j], kind: 'point', point: it.point, origin: 'support', supportId: it.id });
      }
    }
  }

  let changed = true;
  while (changed) {
    changed = false;
    for (let i = 0; i < size; i++) {
      for (let j = i + 1; j < size; j++) {
        if (grid[slot(i, j)]) continue;
        const beam = beams.get(slot(i, j));
        const lines = beam ? [...beam] : [];
        for (let m = 0; m < size; m++) {
          if (m === i || m === j) continue;
          const partnerA = grid[slot(i, m)];
          const partnerB = grid[slot(j, m)];
          if (!partnerA || !partnerB) continue;
          const line = lineThrough(partnerA, partnerB);
          if (line) lines.push({ ...line, source: { type: 'kennedy', through: [partnerA.key, partnerB.key], body: m } });
        }
        const found = intersectFirstPair(lines);
        if (!found) continue;
        add({ key: poleKey(i, j), pair: [i, j], origin: 'derived', ...found });
        changed = true;
      }
    }
  }
  return poles;
}

/** Intersects the first two non-coincident lines; null if there are fewer than two distinct lines. */
function intersectFirstPair(lines) {
  for (let a = 0; a < lines.length; a++) {
    for (let b = a + 1; b < lines.length; b++) {
      const r = intersectLines(lines[a], lines[b]);
      const via = [lines[a], lines[b]];
      if (r.kind === 'point') return { kind: 'point', point: r.point, via };
      if (r.kind === 'parallel') return { kind: 'infinity', direction: r.direction, via };
    }
  }
  return null;
}

/** Only the derived poles, in key order of discovery. */
export const derivedPoles = (poles) => [...poles.values()].filter((p) => p.origin === 'derived');

/**
 * The Kennedy auxiliary lines used for the derived poles: { p, d, through, key, body } where
 * `key` is the derived pole, `through` the two known poles the line connects. (Roller beams used
 * in a derivation are not listed: they are the pole beams drawn anyway.)
 */
export function kennedyLines(poles) {
  const out = [];
  for (const pole of derivedPoles(poles)) {
    for (const line of pole.via) {
      if (line.source.type === 'kennedy') {
        out.push({ p: line.p, d: line.d, through: line.source.through, key: pole.key, body: line.source.body });
      }
    }
  }
  return out;
}
