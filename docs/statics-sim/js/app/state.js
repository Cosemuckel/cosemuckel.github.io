// @ts-check
// The application state and a tiny observable store around it. DOM-free.
//
// The state object is one plain mutable record that the app modules read and write directly (the simulation and pointer
// code touch dozens of fields per frame, so copying on every write would be wasteful). `store.set(patch)` is the path
// for changes other parts of the page need to hear about; `subscribe` registers those listeners.

import * as M from '../core/model.js';
import { DEFAULT_FLAGS, DEFAULT_SCALE } from '../io/serialize.js';
import { defaultView } from './view.js';

/**
 * Wraps `initial` in a store. `store.state` is `initial` itself (mutated in place, never replaced).
 * @template {object} T
 * @param {T} initial
 */
export function createStore(initial) {
  const listeners = new Set();
  return {
    /** The live state object. */
    state: initial,
    /** The live state object (same as `state`; for callers that prefer a function). */
    get: () => initial,
    /**
     * Assigns `patch` onto the state, then calls every listener with `(state, patch)`. A listener that throws stops the
     * remaining listeners and the error reaches the caller of `set`.
     * @param {Partial<T>} patch
     */
    set(patch) {
      Object.assign(initial, patch);
      for (const listener of [...listeners]) listener(initial, patch);
    },
    /**
     * Registers `listener(state, patch)` for every later `set`; returns the function that unregisters it.
     * @param {(state: T, patch: Partial<T>) => void} listener
     */
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
  };
}

/**
 * The initial application state.
 *
 * Simulation (`sim`): `traj` is the finite-motion trajectory rebased at the pose where the user last left it.
 * params/dir/active/locked/ranges are per mode (see sim/trajectory.js): params are display values (the slider
 * positions), offsets[i] the display value at the rebase pose, ranges[i] the display range (min, max, kind).
 * traj = null means "rebase when needed"; stale = another mode moved, so the ranges are re-derived at the
 * current pose (staleTimer) before they are relied on.
 * View: `viewCustom` = the user (or a `cam=` link) moved the view away from "fit", so it goes into shared links;
 * `viewFromLink` = a viewer opened with a valid `cam=`; it keeps the framed radius when its canvas is resized.
 */
export function createAppState() {
  return {
    scene: M.createScene(),
    sys: null,
    q: [],
    analysis: null,
    modeLabels: [], // driver label of each mode slot (see slots.js)
    dq: null,
    items: [],
    intersections: [],
    poles: new Map(),
    tool: 'select',
    view: defaultView(),
    viewCustom: false,
    viewFromLink: false,
    size: { w: 300, h: 300, dpr: 1 },
    opts: { snap: true, ...DEFAULT_FLAGS, scale: DEFAULT_SCALE, mode: 0 },
    sel: null,
    poleSel: new Set(),
    drag: null,
    preview: null,
    chain: null, // open polyline being drawn: { target: body id | null, pts: [...] }
    sim: { on: false, playing: false, traj: null, params: [], offsets: [], dir: [], active: [], locked: [], ranges: null, angleRefs: [], rebaseTimer: 0, stale: false, staleTimer: 0, arc: [], deadCount: [], deadPoint: 'continue', selection: null, angleMemory: new Map(),
      rangeTimer: 0, rangeDone: [], wanted: -1, prior: null },
  };
}

/** The selected body or support of `state` (looked up in the scene), or null when nothing is selected. */
export function findSelected(state) {
  const { sel, scene } = state;
  if (!sel) return null;
  return sel.kind === 'body'
    ? scene.bodies.find((b) => b.id === sel.id)
    : scene.supports.find((s) => s.id === sel.id);
}
