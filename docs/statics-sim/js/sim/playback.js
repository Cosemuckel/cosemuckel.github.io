// @ts-check
// Playback of mode parameters and the mapping between display values and trajectory parameters (pure maths).

/** What a partial driver (slider, rocker) does at a dead point, the end of a monotone arc of its values. */
export const DEAD_POINT_RULES = ['continue', 'reverse', 'random'];

/** Deterministic number in [0, 1) for an integer seed (hash + one mulberry32 step), for the 'random' rule. */
export function seededUnit(seed) {
  let a = (Math.imul(seed + 1, 0x9e3779b1) + 0x6d2b79f5) >>> 0;
  let t = Math.imul(a ^ (a >>> 15), 1 | a);
  t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
  a = (t ^ (t >>> 14)) >>> 0;
  return a / 4294967296;
}

/**
 * The rule applied at one dead point: 'continue' (onto the adjacent arc in the same sense of the configuration walk: the
 * crank keeps turning, the driver reverses) or 'reverse' (back along the same arc: the driver reverses and so does the
 * crank). 'random' picks one with probability 1/2 from the seed (mode index + dead-point counter), so a run is
 * reproducible. Unknown rules count as 'continue'.
 */
export function deadPointChoice(rule, seed) {
  if (rule === 'reverse') return 'reverse';
  if (rule === 'random') return seededUnit(seed) < 0.5 ? 'continue' : 'reverse';
  return 'continue';
}

const wrapIndex = (i, n) => ((i % n) + n) % n;
/** Index of the arc after `arc` in the sense `walk` (+1 / -1) of the curve, or -1: loops are cyclic, open curves end. */
const neighbourArc = (range, arc, walk) => {
  const n = range.arcs.length;
  const next = arc + walk;
  if (range.status === 'closed') return n > 1 ? wrapIndex(next, n) : -1;
  return next >= 0 && next < n ? next : -1;
};
/** True when playback follows the arcs of this mode (both ends are extrema / limits and the arc is known). */
const usesArcs = (range, arc) => range.kind === 'limited' && Array.isArray(range.arcs) && !!range.arcs[arc] && range.arcs[arc].dir !== 0;

/**
 * Starting arc and direction of a limited mode shown at its base pose: { arc, dir, dead }. Normally the arc of the
 * base pose (`branch`) with dir = +1. When the base pose is itself a dead point (the arc starts there) the rule
 * decides the sense of the walk: 'continue' follows the positive arclength sense (onto the arc of the base pose),
 * 'reverse' the negative one (onto the arc that ends there, walked backwards); 'random' uses `seed`. The caller
 * counts a dead start as one dead point (`dead`) when it keeps a counter.
 */
export function startingArc(range, rule, seed) {
  const plain = { arc: range.branch, dir: 1, dead: false };
  if (!usesArcs(range, range.branch) || range.arcs.length < 2) return plain;
  const here = range.arcs[range.branch];
  if (Math.abs(here.from) > 1e-6) return plain;
  const choice = deadPointChoice(rule, seed);
  const prev = choice === 'reverse' ? neighbourArc(range, range.branch, -1) : -1;
  if (prev >= 0 && range.arcs[prev].dir !== 0) return { arc: prev, dir: -range.arcs[prev].dir, dead: true };
  return { arc: range.branch, dir: here.dir, dead: true };
}

/**
 * One playback tick of the mode parameters. `state` = { values, dir (+1 / -1 per mode), playing,
 * locked, ranges, arc?, deadCount?, deadPoint? } (arrays indexed by mode); `ranges[i]` = { min, max, kind } where
 * kind 'closed' also carries `period` = max - min. `ds` is the advance per tick (a number, or one number per mode).
 * Playing, unlocked modes advance and turn round at the ends of their range; locked modes keep their
 * value whatever else is happening. A closed mode wraps from one end to the other (a full revolution
 * loops) as long as it is the only mode moving: with several modes moving together the pose is only
 * defined along one straight walk in parameter space, so a closed mode then turns round like the others.
 *
 * Dead points: when `arc` (the arc index of each mode) is given, a 'limited' mode whose range carries `arcs` (and
 * `status`) moves on its arc and, at the arc's end, follows the rule `deadPoint` ('continue' | 'reverse' | 'random',
 * default 'continue'; see deadPointChoice; `deadCount[i]` counts the dead points passed and seeds 'random'). Continue
 * moves on to the adjacent arc in the same sense of the configuration walk (the value is continuous, its direction
 * reverses); an open curve has no arc beyond its last one, so that end reverses. The result then also carries `arc`,
 * `walk` (+1 / -1: the sense of the configuration walk, = dir x the arc's dir) and `deadCount`.
 */
export function advanceParams({ values, dir, playing, locked, ranges, arc, deadCount, deadPoint = 'continue' }, ds) {
  const nextValues = values.slice();
  const nextDir = dir.slice();
  const nextArc = arc ? arc.slice() : null;
  const nextCount = arc ? values.map((_, i) => (deadCount && deadCount[i]) || 0) : null;
  const moving = values.filter((_, i) => playing[i] && !locked[i]).length;
  values.forEach((v, i) => {
    if (!playing[i] || locked[i]) return;
    const range = ranges[i];
    const { min, max, kind } = range;
    const step = Array.isArray(ds) ? ds[i] : ds;
    let t = v + dir[i] * step;
    if (nextArc && usesArcs(range, nextArc[i])) {
      const current = range.arcs[nextArc[i]];
      const lo = Math.min(current.gFrom, current.gTo);
      const hi = Math.max(current.gFrom, current.gTo);
      if (t > lo && t < hi) {
        nextValues[i] = t;
        return;
      }
      const end = dir[i] > 0 ? hi : lo;
      const walk = dir[i] * current.dir;
      const choice = deadPointChoice(deadPoint, i + nextCount[i]);
      nextCount[i]++;
      const next = choice === 'continue' ? neighbourArc(range, nextArc[i], walk) : -1;
      nextValues[i] = end;
      if (next >= 0 && range.arcs[next].dir !== 0) {
        nextArc[i] = next;
        nextDir[i] = walk * range.arcs[next].dir;
      } else nextDir[i] = -dir[i];
      return;
    }
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
  if (!nextArc) return { values: nextValues, dir: nextDir };
  const walk = nextArc.map((a, i) => (ranges[i] && usesArcs(ranges[i], a) ? nextDir[i] * ranges[i].arcs[a].dir : 0));
  return { values: nextValues, dir: nextDir, arc: nextArc, walk, deadCount: nextCount };
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
