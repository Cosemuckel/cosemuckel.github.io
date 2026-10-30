// Background grid and axes.

import { gridStep } from '../core/geometry.js';
import { COLORS as C } from './theme.js';

export function drawGrid(ctx, F, T) {
  const { w, h, view } = F;
  const step = gridStep(view.scale);
  const x0 = view.cx - w / 2 / view.scale;
  const x1 = view.cx + w / 2 / view.scale;
  const y0 = view.cy - h / 2 / view.scale;
  const y1 = view.cy + h / 2 / view.scale;
  ctx.lineWidth = 1;
  ctx.strokeStyle = C.grid;
  ctx.beginPath();
  for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
    const a = T({ x, y: y0 });
    ctx.moveTo(Math.round(a.x) + 0.5, 0);
    ctx.lineTo(Math.round(a.x) + 0.5, h);
  }
  for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
    const a = T({ x: x0, y });
    ctx.moveTo(0, Math.round(a.y) + 0.5);
    ctx.lineTo(w, Math.round(a.y) + 0.5);
  }
  ctx.stroke();
  ctx.strokeStyle = C.axis;
  ctx.beginPath();
  const o = T({ x: 0, y: 0 });
  ctx.moveTo(Math.round(o.x) + 0.5, 0);
  ctx.lineTo(Math.round(o.x) + 0.5, h);
  ctx.moveTo(0, Math.round(o.y) + 0.5);
  ctx.lineTo(w, Math.round(o.y) + 0.5);
  ctx.stroke();
}
