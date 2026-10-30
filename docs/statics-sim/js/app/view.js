// @ts-check
// The camera: world <-> screen transforms, fitting, zooming and panning. The maths is DOM-free (unit-tested in
// tests/app/view.test.js); only the three canvas helpers at the end touch the DOM.
//
// A view is { cx, cy, scale }: the world point at the canvas centre and pixels per world unit. A size is
// { w, h, dpr } in CSS pixels. Functions return new views and never mutate their arguments; y points up in the world
// and down on the screen.

import * as G from '../core/geometry.js';
import { VIEW_SCALE, WHEEL_ZOOM_RATE } from '../constants.js';
import { decodeView, viewRadius, scaleForRadius } from '../io/viewlink.js';
import { framingText } from '../io/options.js';

/** The view of an empty scene: world centre (cx, cy) and `scale` pixels per world unit. */
export const defaultView = () => ({ cx: 0, cy: 2, scale: 40 });

const FIT_MARGIN = 4; // world units added around the content when fitting
const FIT_SCALE = { min: 8, max: 120 };

/** World point under screen point `sp`. */
export const screenToWorld = (view, size, sp) => ({
  x: (sp.x - size.w / 2) / view.scale + view.cx,
  y: (size.h / 2 - sp.y) / view.scale + view.cy,
});

/** Screen point of world point `p`. */
export const worldToScreen = (view, size, p) => ({
  x: (p.x - view.cx) * view.scale + size.w / 2,
  y: size.h / 2 - (p.y - view.cy) * view.scale,
});

/** `p` rounded to the grid line spacing that is current at this zoom level. */
export function snapToGrid(view, p) {
  const step = G.gridStep(view.scale);
  return { x: G.snapValue(p.x, step), y: G.snapValue(p.y, step) };
}

/** The view that frames every body point and support of `scene` in a canvas of `size` (the default view for an empty scene). */
export function fitViewFor(scene, size) {
  const points = scene.bodies.flatMap((b) => b.pts).concat(scene.supports.map((s) => s.p));
  if (!points.length) return defaultView();
  const xs = points.map((p) => p.x);
  const ys = points.map((p) => p.y);
  const wx = Math.max(...xs) - Math.min(...xs) + FIT_MARGIN;
  const wy = Math.max(...ys) - Math.min(...ys) + FIT_MARGIN;
  return {
    cx: (Math.max(...xs) + Math.min(...xs)) / 2,
    cy: (Math.max(...ys) + Math.min(...ys)) / 2,
    scale: Math.max(FIT_SCALE.min, Math.min(size.w / wx, size.h / wy, FIT_SCALE.max)),
  };
}

/** `view` zoomed by a wheel event of `deltaY` at screen point `sp`; the world point under `sp` stays under it. Scale is clamped to VIEW_SCALE. */
export function zoomAt(view, size, sp, deltaY) {
  const before = screenToWorld(view, size, sp);
  const zoomed = { ...view, scale: Math.min(VIEW_SCALE.max, Math.max(VIEW_SCALE.min, view.scale * Math.exp(-deltaY * WHEEL_ZOOM_RATE))) };
  const after = screenToWorld(zoomed, size, sp);
  return { cx: zoomed.cx + (before.x - after.x), cy: zoomed.cy + (before.y - after.y), scale: zoomed.scale };
}

/** `view` dragged by (dx, dy) screen pixels (the content follows the pointer). */
export const panBy = (view, dx, dy) => ({ cx: view.cx - dx / view.scale, cy: view.cy + dy / view.scale, scale: view.scale });

/** The framing requested by the `cam=` link option (or the legacy `view=`) for a canvas of `size`; null when absent or invalid. */
export function linkView(params, size) {
  const text = framingText(params);
  return text !== null ? decodeView(text, size) : null;
}

/** `view` for a canvas that changed from size `previous` to `size`: the same inscribed world circle stays framed. */
export const keepFramedRadius = (view, previous, size) => ({ ...view, scale: scaleForRadius(viewRadius(view.scale, previous), size) });

/** DOM: the on-screen size of `canvas` in CSS pixels (at least 50 each way) and the device pixel ratio. */
export function measureCanvas(canvas) {
  const r = canvas.getBoundingClientRect();
  return { w: Math.max(50, r.width), h: Math.max(50, r.height), dpr: window.devicePixelRatio || 1 };
}

/** DOM: sets the backing-store resolution of `canvas` for `size` (this clears the canvas). */
export function sizeBackingStore(canvas, size) {
  canvas.width = Math.round(size.w * size.dpr);
  canvas.height = Math.round(size.h * size.dpr);
}

/** DOM: position of a pointer/mouse event relative to the top-left corner of `canvas`, in CSS pixels. */
export function eventPosition(canvas, e) {
  const r = canvas.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}
