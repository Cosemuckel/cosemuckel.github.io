// @ts-check
// Zoom/pan in the URL: the `cam=cx,cy,r` hash option. DOM-free, so it is unit-tested in tests/io/viewlink.test.js.
//
// The view is stored apart from the setup (`s=`, see serialize.js), so panning never re-encodes the scene.
// cx, cy are the world coordinates of the view centre; r is the world radius of the circle inscribed in the
// canvas (r = min(w, h) / 2 / scale). A viewer rebuilds scale = min(w, h) / 2 / r for its own canvas size, so an
// iframe of another size frames the same central circle instead of the same scale.
//
// Note: `view=1` / `view=on` is the viewer flag (see isViewer in options.js), so the framing has its own key. The
// first version of this feature wrote `view=cx,cy,r`; that spelling is still read (see framingText) but never written.

import { LIMITS } from './serialize.js';
import { VIEW_SCALE } from '../constants.js';
import { optionFlag, CAM_KEY, LEGACY_KEY } from './options.js';

/** Accepted world radius (exclusive bounds); anything else in a link is ignored. */
const RADIUS_LIMITS = { min: 1e-3, max: 1e4 };

const NUMBER = /^[+-]?(\d+\.?\d*|\.\d+)(e[+-]?\d+)?$/i; // rejects '', '0x10', 'Infinity', 'NaN'

const inscribedDiameter = (size) => Math.min(size.w, size.h);
/** World radius of the circle inscribed in a canvas of `size` at `scale` px per unit. */
export const viewRadius = (scale, size) => inscribedDiameter(size) / 2 / scale;
/** Scale at which a canvas of `size` shows the inscribed world radius `r`. */
export const scaleForRadius = (r, size) => inscribedDiameter(size) / 2 / r;

function checkedParts([cx, cy, r]) {
  const ok = Math.abs(cx) <= LIMITS.coord && Math.abs(cy) <= LIMITS.coord && r > RADIUS_LIMITS.min && r < RADIUS_LIMITS.max;
  return ok ? { cx, cy, r } : null;
}

/**
 * `view` ({ cx, cy, scale }) + canvas size -> "cx,cy,r" with cx, cy rounded to 0.01 and r to 3 significant
 * digits, or null when the view cannot be written (non-finite numbers, or values decodeView would refuse).
 */
export function encodeView(view, size) {
  const r = viewRadius(view.scale, size);
  if (![view.cx, view.cy, r].every(Number.isFinite)) return null;
  const parts = [Number(view.cx.toFixed(2)), Number(view.cy.toFixed(2)), Number(r.toPrecision(3))];
  return checkedParts(parts) ? parts.join(',') : null;
}

/**
 * "cx,cy,r" + canvas size -> { cx, cy, scale } (scale clamped to VIEW_SCALE), or null for anything that is not
 * exactly three plain finite numbers within bounds (null, garbage, NaN, huge, zero or negative r, ...).
 */
export function decodeView(text, size) {
  if (typeof text !== 'string' || text.length > 100) return null;
  const parts = text.split(',').map((p) => p.trim());
  if (parts.length !== 3 || !parts.every((p) => NUMBER.test(p))) return null;
  const v = checkedParts(parts.map(Number));
  if (!v) return null;
  const scale = Math.min(VIEW_SCALE.max, Math.max(VIEW_SCALE.min, scaleForRadius(v.r, size)));
  return { cx: v.cx, cy: v.cy, scale };
}

/**
 * Hash text (without "#") for `hash` (string or URLSearchParams) with `cam` set to `viewText`, or removed when
 * `viewText` is null. A legacy `view=cx,cy,r` framing is dropped (it is replaced by `cam`), while the on/off
 * `view` flag (`view=1`) is not a framing and always survives. Commas stay unescaped so the link reads `cam=1.5,2,6`.
 */
export function hashWithView(hash, viewText) {
  const p = new URLSearchParams(hash);
  if (optionFlag(p, LEGACY_KEY) === undefined) p.delete(LEGACY_KEY); // a flag stays; a triple or garbage goes
  if (viewText) p.set(CAM_KEY, viewText);
  else p.delete(CAM_KEY);
  return p.toString().replace(/%2C/gi, ',');
}
