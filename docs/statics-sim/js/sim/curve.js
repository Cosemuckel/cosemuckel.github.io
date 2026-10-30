// @ts-check
// One-degree-of-freedom configuration curves: walking, arcs, winding and range classification (pure maths).
// The stepping primitives and the search budget live in curve-walk.js.
//
// A curve is followed from a start pose in fixed steps, each a tangent step followed by Newton correction. The
// tangent is tracked by projecting the previous one onto the null space of the current pose (continuity of the sense
// of motion, so dead points of a crank are passed). Positions along the curve are distances s travelled by the
// fastest body point; s > 0 is the sense of the start tangent.
//
// classify() answers, for one curve and one driver coordinate g(q):
//   closed   the curve comes back to its start pose (same point, same sense of motion): period = length of the loop.
//            The driver's WINDING w = round((g_end - g_start) / 2 pi) says how often it turned round on the loop; a
//            driver with w != 0 is a "full revolution" driver, any other driver just swings (driverKind 'limited').
//   blocked  the walk cannot continue (limit / singular position)
//   free     the walk keeps moving away without bound (a free roller, a pure translation)
//   budget   the search budget ended before any of the above could be established ("not fully explored")
// `arcs` are the monotone pieces of g along the curve, split at the extrema of g (located by checkpoints and refined
// by bisection on the sign of dg/ds); `branch` is the arc that contains the start pose.

import { normalizeMode } from '../core/kinematics.js';
import { runSync } from './task.js';
import { STEP, MAX_WALK } from '../constants.js';
import { advance, trackBasis, poseDistance, bodySpeed, dotv, MIN_NORM } from './curve-walk.js';

const LEAVE = 0.5; // a walk counts as having left the base pose beyond this pose distance (x step scale)
const CLOSE = 0.25; // ... and as possibly returning when it comes closer than this again (x step scale)
const CLOSED_TOL = 1e-4; // refined return distance accepted as "back at the base pose"
const SAME_SENSE = 0.9; // cosine of the angle between the tangents of a genuine return
const FREE_FRACTION = 0.5; // a walk is free when it ends at least this fraction of the searched length away ...
const FREE_SIZES = 2; // ... and at least this many mechanism diagonals away (a loop never gets that far from its start)
const EXTREMUM_TOL = 1e-7; // bisection width (curve distance) for extrema of the driver
const FLAT = 1e-9; // changes of the driver below this (relative) count as flat when looking for extrema
const DEAD_RATE = 1e-7; // |dg/ds| below this at the start pose: the start is a dead point of the driver

/**
 * One-degree-of-freedom curve of `sys` through `start` = { q, tangent } (a null-space vector there, whose sign is
 * the positive sense) with the search budget `budget` (see searchBudget).
 *   poseAt(s)     { q, reached, blocked }: pose at signed distance s; when blocked first (limit / singular
 *                 position, or beyond the walk limit) q is the last reachable pose and `reached` the distance reached
 *   classify(spec)  see classifyResult below. spec = { driver?, candidates? }: `driver` is the coordinate whose
 *                 winding and arcs are reported; without it the best of `candidates` is chosen (a winding rotation
 *                 first, otherwise the first one that changes along the curve at all).
 *   classifySteps(spec)  the same computation as a generator that yields between steps of the walk (see task.js), so a
 *                 caller can slice it in time; classify() is runSync(classifySteps()) and gives identical results.
 */
export function createCurve(sys, start, budget) {
  const { step, maxSteps, scale } = budget;
  const walkLimit = Math.max(MAX_WALK, budget.searchLength);
  const origin = { q: start.q.slice(), basis: [start.tangent.slice()] };
  const dq0 = normalizeMode(sys, start.q, start.tangent) || start.tangent;
  const startTangent = (() => {
    const l = Math.sqrt(dotv(start.tangent, start.tangent));
    return start.tangent.map((v) => v / l);
  })();
  let loopPeriod = null; // forward distance of the closed loop once classify has found it
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
    const L = Math.min(full, walkLimit); // longer walks are cut off (reported as blocked)
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
    return good * step + Math.max(0, lo - 1e-4 * Math.min(1, step / STEP));
  }

  /**
   * Where the walk is back at the start pose, searched around checkpoint c: { at, state } or null. The return counts
   * only at the same point of the curve in the same sense of motion (a pose crossed in another direction is a
   * crossing of two branches, not a closed loop).
   */
  function* refineReturn(walk, c) {
    let best = { d: Infinity, at: 0, state: null };
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
        yield;
      }
      const h = (lo + hi) / 2;
      const state = advance(sys, walk.states[m], [walk.sign * h]);
      const d = state ? poseDistance(sys, start.q, state.q) : Infinity;
      if (d < best.d) best = { d, at: m * step + h, state };
    }
    if (!(best.d < CLOSED_TOL)) return null;
    const tangent = trackBasis(sys, best.state);
    if (!tangent || dotv(tangent[0], startTangent) < SAME_SENSE) return null;
    return { at: best.at, state: best.state };
  }

  /**
   * Walks in one direction: { type: 'closed', period, state } | { type: 'blocked', reach } | { type: 'free' } |
   * { type: 'budget' } (neither blocked, closed nor clearly running away within the budget).
   */
  const scans = new Map(); // sign -> scan result: it depends on the walk only, not on the driver being classified
  const scanning = new Map(); // sign -> the running scanWalk generator, shared by every classification that needs it
  function* scan(sign, detectReturn) {
    if (!scanning.has(sign)) scanning.set(sign, scanWalk(sign, detectReturn));
    const running = scanning.get(sign);
    while (!scans.has(sign)) {
      const step = running.next();
      if (step.done) scans.set(sign, step.value);
      else yield;
    }
    return scans.get(sign);
  }
  function* scanWalk(sign, detectReturn) {
    const walk = walks.get(sign);
    const leave = LEAVE * scale;
    const close = CLOSE * scale;
    const dist = [0];
    let left = false;
    for (let n = 1; n <= maxSteps; n++) {
      extend(walk, n);
      if (walk.states.length - 1 < n) return { type: 'blocked', reach: reachOfBlockedWalk(walk) };
      dist.push(poseDistance(sys, start.q, walk.states[n].q));
      if (dist[n] > leave) left = true;
      // dist[n - 1] was a local minimum near the start pose: the walk may be coming back round.
      if (detectReturn && left && n >= 2 && dist[n - 1] < close && dist[n] >= dist[n - 1]) {
        const back = yield* refineReturn(walk, n - 1);
        if (back) return { type: 'closed', period: back.at, state: back.state };
      }
      yield;
    }
    const away = Math.max(FREE_FRACTION * maxSteps * step, FREE_SIZES * (budget.diagonal || 0));
    return { type: dist[maxSteps] >= away ? 'free' : 'budget' };
  }

  // ----------------------------------------------------------- driver analysis

  const rateOf = (driver) => dotv(driver.gradient(start.q), dq0);
  /** Speed used to prefer one winding driver over another: the body that moves most (as it always did). */
  const speedOf = (driver) =>
    driver.kind === 'rotation' ? bodySpeed(start.q, dq0, driver.body, sys.bodies[driver.body]) : Math.abs(rateOf(driver));
  const windingOf = (driver, endQ) =>
    driver.kind === 'rotation' || driver.kind === 'relAngle' ? Math.round((driver.value(endQ) - driver.value(start.q)) / (2 * Math.PI)) || 0 : 0;

  /** Sign of dg/ds at a walk state (0 where the driver is momentarily flat or the tangent degenerates). */
  function slopeSign(driver, state) {
    const basis = trackBasis(sys, state);
    if (!basis) return 0;
    const rate = dotv(driver.gradient(state.q), basis[0]);
    return Math.abs(rate) < 1e-12 ? 0 : Math.sign(rate);
  }

  /** Extremum of g between two adjacent samples whose slopes differ: { s, g, q }. */
  function* refineExtremumSteps(driver, a, b) {
    const [near, far] = Math.abs(a.s) <= Math.abs(b.s) ? [a, b] : [b, a];
    const dir = far.s > near.s ? 1 : -1;
    const slopeNear = slopeSign(driver, near.state);
    let lo = 0;
    let hi = Math.abs(far.s - near.s);
    let best = near.state;
    for (let k = 0; k < 60 && hi - lo > EXTREMUM_TOL; k++) {
      const mid = (lo + hi) / 2;
      const trial = advance(sys, near.state, [dir * mid]);
      if (!trial) break;
      if (slopeSign(driver, trial) === slopeNear) {
        lo = mid;
        best = trial;
      } else hi = mid;
      yield;
    }
    return { s: near.s + (dir * (lo + hi)) / 2, g: driver.value(best.q), q: best.q };
  }

  const refineExtremum = (driver, a, b) => runSync(refineExtremumSteps(driver, a, b));

  /**
   * Arcs / extent of `driver` over `samples` (ascending s, each { s, state }). `loop` = the curve is closed, so the
   * first and last arc continue each other through the start pose.
   */
  function* driverArcs(driver, samples, loop) {
    const g = samples.map((p) => driver.value(p.state.q));
    const flat = FLAT * (1 + Math.max(...g.map(Math.abs)));
    const extrema = [];
    let last = { sign: 0, index: -1 };
    for (let i = 0; i + 1 < samples.length; i++) {
      const d = g[i + 1] - g[i];
      const sign = Math.abs(d) <= flat ? 0 : Math.sign(d);
      if (!sign) continue;
      if (last.sign && sign !== last.sign) {
        const lo = last.index;
        const hi = i + 1;
        if (hi - lo === 2) {
          const mid = lo + 1;
          // The slope at `mid` tells which half of the bracket holds the turning point.
          const [from, to] = slopeSign(driver, samples[mid].state) === last.sign ? [mid, hi] : [lo, mid];
          extrema.push(yield* refineExtremumSteps(driver, samples[from], samples[to]));
        } else {
          const mid = samples[Math.floor((lo + hi) / 2)];
          extrema.push({ s: mid.s, g: driver.value(mid.state.q), q: mid.state.q });
        }
      }
      last = { sign, index: i };
    }
    const first = samples[0];
    const end = samples[samples.length - 1];
    const cuts = [{ s: first.s, g: g[0] }, ...extrema, { s: end.s, g: g[g.length - 1] }];
    let arcs = [];
    for (let i = 0; i + 1 < cuts.length; i++) {
      const [p, r] = [cuts[i], cuts[i + 1]];
      if (r.s - p.s < 1e-9) continue;
      arcs.push({ from: p.s, to: r.s, gFrom: p.g, gTo: r.g, dir: Math.sign(r.g - p.g) });
    }
    if (loop && arcs.length > 1 && arcs[0].dir === arcs[arcs.length - 1].dir) {
      const lastArc = arcs[arcs.length - 1];
      const wrapped = { from: lastArc.from - end.s, to: arcs[0].to, gFrom: lastArc.gFrom - (g[g.length - 1] - g[0]), gTo: arcs[0].gTo, dir: lastArc.dir };
      arcs = [wrapped, ...arcs.slice(1, -1)];
    }
    const values = [...g, ...extrema.map((e) => e.g)];
    const eps = 1e-9;
    return {
      arcs,
      branch: arcs.findIndex((a) => a.from <= eps && a.to > eps),
      extent: { min: Math.min(...values), max: Math.max(...values) },
    };
  }

  /** Samples of a walk: [{ s, state }] for n = 0..count along `sign`. */
  const walkSamples = (walk, count) => walk.states.slice(0, count + 1).map((state, n) => ({ s: walk.sign * n * step, state }));

  // ----------------------------------------------------------- driver-valued parametrisation

  /**
   * Pose at which `driver` has changed by `delta` from its value at the start pose, on the monotone arc of the
   * driver that contains the start pose: the walk goes outward from the start in the sense that moves the driver
   * towards the target and stops at the first extremum of the driver (or limit of the curve), so the result never
   * jumps to another branch of the curve. Returns { q, delta, blocked }: `delta` is the change actually reached
   * (equal to the requested one unless `blocked`, which means the arc, the curve or the walk limit ended first).
   * A pure function of (curve, driver, delta): no state is kept between calls except the cached walk checkpoints.
   * Where the start pose is a dead point of the driver (|dg/ds| ~ 0: a slider at its stroke limit) and both senses of
   * the walk move the driver towards the target, `deadSense` (+1 = the positive arclength sense, -1 = the negative one;
   * the dead-point rule of the playback decides it) says which one is taken; otherwise the sense that moves the driver
   * towards the target wins (the larger first change, the positive one on a tie).
   */
  function solveDriver(driver, delta, { deadSense = 1 } = {}) {
    const g0 = driver.value(start.q);
    const here = { q: start.q.slice(), delta: 0, blocked: false };
    if (Math.abs(delta) < 1e-14) return here;
    const e = Math.sign(delta);
    const target = g0 + delta;
    const tol = 1e-12 * (1 + Math.abs(target));
    const reachTol = 1e-9 * (1 + Math.abs(target));
    const toward = (g) => (g - target) * e; // negative until the target is reached

    /** Change of the driver towards the target after the first step (or up to the limit) in the given sense. */
    const probe = (sign) => {
      const walk = walks.get(sign);
      extend(walk, 1);
      const state = walk.states.length > 1 ? walk.states[1] : advance(sys, walk.states[0], [sign * reachOfBlockedWalk(walk)]);
      return state ? (driver.value(state.q) - g0) * e : -Infinity;
    };
    const firsts = [probe(1), probe(-1)];
    const flat = FLAT * (1 + Math.abs(g0));
    if (Math.max(...firsts) <= flat) return { ...here, blocked: true };
    const dead = Math.abs(rateOf(driver)) < DEAD_RATE && Math.min(...firsts) > flat;
    const dir = dead ? (deadSense < 0 ? -1 : 1) : firsts[0] >= firsts[1] ? 1 : -1;

    const walk = walks.get(dir);
    const stateAt = (m, h) => advance(sys, walk.states[m], [dir * h]);
    const clampAt = (state) => ({ q: state.q.slice(), delta: driver.value(state.q) - g0, blocked: true });

    /** Solves driver = target on the piece starting at checkpoint m, h in [0, hEnd] (regula falsi, Illinois variant). */
    function solveWithin(m, hEnd) {
      const begin = walk.states[m];
      const end = stateAt(m, hEnd);
      if (!end) return clampAt(begin);
      let fb = toward(driver.value(end.q));
      // The ends of an arc are extrema located to ~1e-7 in distance: a target this close counts as reached there.
      if (fb < -reachTol) return clampAt(end);
      if (fb <= tol) return { q: end.q.slice(), delta, blocked: false };
      let a = 0;
      let b = hEnd;
      let fa = toward(driver.value(begin.q));
      let side = 0;
      let best = end;
      for (let it = 0; it < 100; it++) {
        let c = fb === fa ? (a + b) / 2 : b - (fb * (b - a)) / (fb - fa);
        if (!(c > a && c < b)) c = (a + b) / 2;
        const trial = stateAt(m, c);
        if (!trial) {
          b = c;
          fb = Math.abs(fa);
          continue;
        }
        best = trial;
        const fc = toward(driver.value(trial.q));
        if (Math.abs(fc) <= tol || b - a < 1e-15) break;
        if (fc < 0) {
          a = c;
          fa = fc;
          if (side === -1) fb /= 2;
          side = -1;
        } else {
          b = c;
          fb = fc;
          if (side === 1) fa /= 2;
          side = 1;
        }
      }
      return { q: best.q.slice(), delta, blocked: false };
    }

    const maxN = Math.floor(walkLimit / step + 1e-9);
    let previous = g0;
    for (let n = 1; n <= maxN; n++) {
      extend(walk, n);
      if (walk.states.length - 1 < n) {
        const m = walk.states.length - 1;
        return solveWithin(m, Math.max(reachOfBlockedWalk(walk) - m * step, 0));
      }
      const g = driver.value(walk.states[n].q);
      if (toward(g) >= 0) return solveWithin(n - 1, step);
      if ((g - previous) * e < -FLAT * (1 + Math.abs(g))) {
        // The driver turned round between checkpoints n - 2 and n: find the extremum and stop there.
        const stillToward = dir * slopeSign(driver, walk.states[n - 1]) === e;
        const lower = stillToward || n === 1 ? n - 1 : n - 2;
        const ext = refineExtremum(
          driver,
          { s: dir * lower * step, state: walk.states[lower] },
          { s: dir * (lower + 1) * step, state: walk.states[lower + 1] });
        return solveWithin(lower, Math.min(step, Math.abs(ext.s - dir * lower * step)));
      }
      previous = g;
    }
    return clampAt(walk.states[Math.min(maxN, walk.states.length - 1)]);
  }

  /**
   * Pose at which `driver` has the value `value` on the given arc (one of the arcs classify reports: a monotone piece
   * of the driver on the curve, `from`..`to` in curve distance), found by bracketing on the arc: the result lies on
   * that arc whatever else the curve does. Returns { q, delta, blocked } like solveDriver (`delta` = change from the
   * start pose); a value beyond the arc is clamped to its end with blocked = true.
   */
  function solveOnArc(driver, arc, value) {
    const g0 = driver.value(start.q);
    const lo = Math.min(arc.gFrom, arc.gTo);
    const hi = Math.max(arc.gFrom, arc.gTo);
    const tol = 1e-9 * (1 + Math.abs(value));
    // The walk distance is only a step count (it differs a little between the forward and the backward walk), so an
    // arc of a closed curve that runs through the start pose (negative `from`) is followed forwards on both sides:
    // its first part ends at the forward distance `loopPeriod`, where the walk is back at the start pose.
    const wrapped = loopPeriod !== null && arc.from < 0;
    const pieces = wrapped
      ? [{ a: arc.from + loopPeriod, b: loopPeriod, ga: arc.gFrom, gb: driver.value(poseAt(loopPeriod).q) },
        { a: 0, b: arc.to, ga: g0, gb: arc.gTo }]
      : [{ a: arc.from, b: arc.to, ga: arc.gFrom, gb: arc.gTo }];
    const sFrom = pieces[0].a;
    const sTo = pieces[pieces.length - 1].b;
    const at = (s, blocked) => {
      const q = poseAt(s).q;
      return { q, delta: driver.value(q) - g0, blocked };
    };
    if (value < lo - tol || value > hi + tol) return at((value < lo ? lo : hi) === arc.gFrom ? sFrom : sTo, true);
    if (Math.abs(value - arc.gFrom) <= tol) return at(sFrom, false);
    if (Math.abs(value - arc.gTo) <= tol) return at(sTo, false);
    const within = (p) => value >= Math.min(p.ga, p.gb) - tol && value <= Math.max(p.ga, p.gb) + tol;
    const piece = pieces.find(within) || pieces[pieces.length - 1];
    let { a, b } = piece;
    let fa = piece.ga - value;
    let fb = piece.gb - value;
    let best = null;
    let side = 0;
    for (let it = 0; it < 80; it++) {
      let c = fb === fa ? (a + b) / 2 : b - (fb * (b - a)) / (fb - fa);
      if (!(c > Math.min(a, b) && c < Math.max(a, b))) c = (a + b) / 2;
      best = poseAt(c);
      const fc = driver.value(best.q) - value;
      if (Math.abs(fc) <= 1e-12 * (1 + Math.abs(value)) || Math.abs(b - a) < 1e-14) break;
      if (Math.sign(fc) === Math.sign(fa)) {
        a = c;
        fa = fc;
        if (side === -1) fb /= 2;
        side = -1;
      } else {
        b = c;
        fb = fc;
        if (side === 1) fa /= 2;
        side = 1;
      }
    }
    return { q: best.q, delta: driver.value(best.q) - g0, blocked: false };
  }

  const NO_DRIVER = { driver: null, winding: 0, arcs: [], branch: -1, extent: null };

  function* closedResult(pos, spec, searched) {
    const endQ = pos.state.q;
    loopPeriod = pos.period;
    const walk = walks.get(1);
    const candidates = spec.driver ? [spec.driver] : spec.candidates || [];
    let driver = spec.driver || null;
    if (!driver) {
      let best = null;
      let bestSpeed = 0;
      for (const d of candidates) {
        if (d.kind !== 'rotation' || windingOf(d, endQ) === 0) continue;
        const speed = speedOf(d);
        if (!best || speed > bestSpeed) {
          best = d;
          bestSpeed = speed;
        }
      }
      driver = best || candidates.find((d) => Math.abs(rateOf(d)) > MIN_NORM) || null;
    }
    const base = { status: 'closed', period: pos.period, reachNeg: null, reachPos: null, sides: { neg: 'closed', pos: 'closed' }, searched };
    if (!driver) return { ...base, ...NO_DRIVER, driverKind: 'closed' };
    const winding = windingOf(driver, endQ);
    const last = Math.min(walk.states.length - 1, Math.floor(pos.period / step + 1e-9));
    const samples = walkSamples(walk, last);
    if (pos.period - last * step > 1e-9) samples.push({ s: pos.period, state: pos.state });
    const { arcs, branch, extent } = yield* driverArcs(driver, samples, true);
    // A closed curve is a full revolution of the driver only when the driver winds round; otherwise it swings.
    return { ...base, driver, winding, arcs, branch, extent, driverKind: winding !== 0 ? 'closed' : 'limited' };
  }

  function* openResult(pos, neg, spec, searched) {
    const blockedPos = pos.type === 'blocked';
    const blockedNeg = neg.type === 'blocked';
    const sides = { neg: neg.type, pos: pos.type };
    const status = blockedPos && blockedNeg ? 'blocked' : sides.neg === 'budget' || sides.pos === 'budget' ? 'budget' : 'free';
    const base = {
      status,
      period: null,
      reachNeg: blockedNeg ? neg.reach : null,
      reachPos: blockedPos ? pos.reach : null,
      sides,
      searched,
      driverKind: blockedPos && blockedNeg ? 'limited' : blockedPos || blockedNeg ? 'half' : 'unbounded',
    };
    const reachSample = (walk, reach) => {
      const good = walk.states.length - 1;
      const state = advance(sys, walk.states[good], [walk.sign * (reach - good * step)]);
      return state ? [{ s: walk.sign * reach, state }] : [];
    };
    const forward = walks.get(1);
    const backward = walks.get(-1);
    const samples = [
      ...(blockedNeg ? reachSample(backward, neg.reach).reverse() : []),
      ...walkSamples(backward, backward.states.length - 1).slice(1).reverse(),
      ...walkSamples(forward, forward.states.length - 1),
      ...(blockedPos ? reachSample(forward, pos.reach) : []),
    ].sort((p, r) => p.s - r.s);
    // Default driver: among the coordinates that move at all, one that is monotone along the whole explored curve
    // comes first (a "full" driver), otherwise the first in preference order (a partial driver).
    let driver = spec.driver || null;
    if (!driver) {
      const moving = (spec.candidates || []).filter((d) => Math.abs(rateOf(d)) > MIN_NORM);
      let monotone = null;
      for (const d of moving) {
        if ((yield* driverArcs(d, samples, false)).arcs.length === 1) {
          monotone = d;
          break;
        }
      }
      driver = monotone || moving[0] || null;
    }
    if (!driver) return { ...base, ...NO_DRIVER };
    const { arcs, branch, extent } = yield* driverArcs(driver, samples, false);
    return { ...base, driver, winding: 0, arcs, branch, extent };
  }

  function* classifySteps(spec = {}) {
    const searched = maxSteps * step;
    const pos = yield* scan(1, true);
    if (pos.type === 'closed') return yield* closedResult(pos, spec, searched);
    const neg = yield* scan(-1, false);
    return yield* openResult(pos, neg, spec, searched);
  }
  const classify = (spec = {}) => runSync(classifySteps(spec));

  return { poseAt, classify, classifySteps, solveDriver, solveOnArc, budget };
}
