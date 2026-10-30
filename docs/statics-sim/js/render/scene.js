// Canvas rendering. Reads a "frame" object, draws, keeps no state of its own.
//
// frame = {
//   w, h, dpr, view: {cx, cy, scale},        canvas size (CSS px) and camera (world centre, px per unit)
//   scene, sys, q, dq, items,                 model, constraint system, pose, unit mode (or null), pole items
//   intersections, poles,                     result of selectedIntersections(), derivePoles() map
//   opts, sel, poleSel, preview,              UI options / selection / drawing preview
// }

import { add, mul } from '../core/geometry.js';
import { worldPoint, pointVelocity } from '../core/kinematics.js';
import { COLORS as C } from './theme.js';
import { worldToScreen } from './screen.js';
import { drawGrid } from './grid.js';
import { drawBodies, drawSupports, drawVectors, drawPreview } from './bodies.js';
import { drawPoleItems, drawKennedyLines, drawIntersections, drawDerivedPoles } from './poles.js';

/** Draws one frame (layers from the bottom: grid, bodies, pole beams, Kennedy lines, supports, poles, vectors, preview). */
export function drawScene(ctx, F) {
  const { w, h, view, opts } = F;
  const T = worldToScreen(view, w, h);

  ctx.setTransform(F.dpr, 0, 0, F.dpr, 0, 0);
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, w, h);
  if (opts.grid) drawGrid(ctx, F, T);

  const q = F.q;
  const cur = F.sys.bodies.map((b, i) => b.pts.map((p) => worldPoint(q, i, p)));
  const showDisp = opts.displaced && F.dq;
  const s = opts.scale;
  const disp = showDisp
    ? cur.map((pts, i) => pts.map((P) => add(P, mul(pointVelocity(q, F.dq, i, P), s))))
    : null;

  drawBodies(ctx, F, T, cur, disp);
  drawPoleItems(ctx, F, T);
  if (opts.inters && opts.kennedy) drawKennedyLines(ctx, F, T);
  drawSupports(ctx, F, T, q);
  if (opts.inters) {
    drawIntersections(ctx, F, T);
    drawDerivedPoles(ctx, F, T);
  }
  if (showDisp && opts.vectors) drawVectors(ctx, T, cur, disp);
  if (F.preview) drawPreview(ctx, T, F.preview);
}
