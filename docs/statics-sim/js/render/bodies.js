// Bodies, supports, displacement vectors and the tool preview.

import { sub, add, mul, len } from '../core/geometry.js';
import { supportPoint } from '../core/kinematics.js';
import { bodySegments, nearestPointOnBody } from '../core/model.js';
import { COLORS as C } from './theme.js';
import { labelText } from './labels.js';

/** Bodies in their current pose `cur` (world points per body), or displaced by `disp` when the overlay is on. */
export function drawBodies(ctx, F, T, cur, disp) {
  const { opts } = F;
  F.scene.bodies.forEach((b, i) => {
    const selected = F.sel && F.sel.kind === 'body' && F.sel.id === b.id;
    if (selected) drawShape(ctx, T, b.kind, cur[i], b.segs, { stroke: C.select, width: 9 });
    if (disp) {
      if (opts.original) drawShape(ctx, T, b.kind, cur[i], b.segs, { stroke: C.ghost, width: 2, dash: [5, 4] });
      drawShape(ctx, T, b.kind, disp[i], b.segs, { stroke: C.displaced, width: 4 });
    } else {
      drawShape(ctx, T, b.kind, cur[i], b.segs, { stroke: C.body, width: 4 });
    }
    if (opts.bodyLabels) {
      // Body number (the ground is body 0), on the body: a rod's label sits on its point closest to the centroid.
      const mid = centroid(cur[i]);
      const at = nearestPointOnBody({ kind: b.kind, pts: cur[i], segs: b.segs }, mid);
      const c = T(at);
      labelText(ctx, String(i + 1), c.x, c.y - 9, { color: C.body, font: 'bold 12px system-ui, sans-serif', align: 'center' });
    }
  });
}

/** Supports at their current positions. */
export function drawSupports(ctx, F, T, q) {
  for (const sp of F.scene.supports) {
    const info = F.sys.supportInfo.get(sp.id);
    const P = T(supportPoint(F.sys, q, sp));
    const selected = F.sel && F.sel.kind === 'support' && F.sel.id === sp.id;
    drawSupport(ctx, sp, P, info && info.active, selected);
  }
}

/** Displacement vectors from each point of `cur` to the same point of `disp`. */
export function drawVectors(ctx, T, cur, disp) {
  ctx.strokeStyle = ctx.fillStyle = C.displaced;
  ctx.lineWidth = 1.5;
  cur.forEach((pts, i) => pts.forEach((P, k) => arrow(ctx, T(P), T(disp[i][k]))));
}

/** The line / polyline being drawn with the current tool. */
export function drawPreview(ctx, T, pv) {
    ctx.strokeStyle = C.select;
    ctx.lineWidth = 3;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    if (pv.kind === 'line') {
      const a = T(pv.a);
      const b = T(pv.b);
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
    } else if (pv.pts.length) {
      pv.pts.forEach((p, k) => {
        const t = T(p);
        if (k === 0) ctx.moveTo(t.x, t.y);
        else ctx.lineTo(t.x, t.y);
      });
    }
    ctx.stroke();
    ctx.setLineDash([]);
}

function centroid(pts) {
  const sum = pts.reduce((a, p) => add(a, p), { x: 0, y: 0 });
  return mul(sum, 1 / pts.length);
}

function drawShape(ctx, T, kind, worldPts, segs, { stroke, width, dash }) {
  const pts = worldPts.map(T);
  ctx.beginPath();
  for (const [i, j] of bodySegments(kind, pts, segs)) {
    ctx.moveTo(pts[i].x, pts[i].y);
    ctx.lineTo(pts[j].x, pts[j].y);
  }
  ctx.setLineDash(dash || []);
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.strokeStyle = stroke;
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawSupport(ctx, sp, P, active, selected) {
  const col = active ? C.support : '#b0b7c0';
  ctx.lineWidth = 2;
  ctx.strokeStyle = col;
  ctx.fillStyle = '#fff';
  if (selected) {
    ctx.strokeStyle = C.select;
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.arc(P.x, P.y, 16, 0, Math.PI * 2);
    ctx.stroke();
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
  }
  if (sp.type === 'pin') {
    ctx.beginPath(); // triangle below the point + ground line
    ctx.moveTo(P.x, P.y);
    ctx.lineTo(P.x - 10, P.y + 16);
    ctx.lineTo(P.x + 10, P.y + 16);
    ctx.closePath();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(P.x - 14, P.y + 16);
    ctx.lineTo(P.x + 14, P.y + 16);
    ctx.stroke();
    if (sp.clamped) {
      ctx.fillStyle = col;
      ctx.fillRect(P.x - 14, P.y + 16, 28, 4);
    }
  } else if (sp.type === 'roller') {
    const a = (sp.angleDeg * Math.PI) / 180;
    const d = { x: Math.cos(a), y: -Math.sin(a) };
    ctx.beginPath(); // rail
    ctx.moveTo(P.x - d.x * 20, P.y - d.y * 20);
    ctx.lineTo(P.x + d.x * 20, P.y + d.y * 20);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(P.x, P.y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    if (sp.locked) {
      ctx.fillStyle = col;
      ctx.font = 'bold 10px system-ui, sans-serif';
      ctx.textAlign = 'left';
      ctx.fillText('locked', P.x + 10, P.y + 18);
    }
  }
  // all supports mark their point; joints are just the open circle
  ctx.beginPath();
  ctx.arc(P.x, P.y, sp.type === 'joint' ? 6 : 3.5, 0, Math.PI * 2);
  ctx.fillStyle = sp.type === 'joint' ? '#fff' : col;
  ctx.fill();
  ctx.stroke();
}

export function arrow(ctx, a, b) {
  const d = sub(b, a);
  const l = len(d);
  if (l < 2) return;
  const u = mul(d, 1 / l);
  const hl = Math.min(8, l * 0.5);
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(b.x, b.y);
  ctx.lineTo(b.x - u.x * hl - u.y * hl * 0.5, b.y - u.y * hl + u.x * hl * 0.5);
  ctx.lineTo(b.x - u.x * hl + u.y * hl * 0.5, b.y - u.y * hl - u.x * hl * 0.5);
  ctx.closePath();
  ctx.fill();
}
