// Pole beams, Kennedy auxiliary lines, picked intersections and derived poles.

import { sub, len, clipRayToRect } from '../core/geometry.js';
import { derivedPoles, kennedyLines, derivedLabel, poleName } from '../core/kennedy.js';
import { COLORS as C } from './theme.js';
import { screenDir as sDir } from './screen.js';
import { labelText } from './labels.js';
import { arrow } from './bodies.js';

const LONG_LINE = 1e4; // half length (px) of a pole beam / auxiliary line
const LINE_MARGIN = 20; // px kept around the canvas so that caps and dashes at its edge are still drawn

/**
 * Adds the line through P with screen direction d (+-LONG_LINE px) to the current path, cut to the canvas rectangle
 * w x h plus a margin. A dashed 20000 px line costs the rasteriser thousands of dashes per beam and frame although only
 * a few hundred px show, so the stroke starts at the dash period (`period` = dash + gap) boundary at or before the
 * cut: the dashes drawn are the same ones as for the uncut line.
 */
export function traceLongLine(ctx, P, d, w, h, period) {
  const norm = Math.hypot(d.x, d.y);
  if (!(norm > 0) || !Number.isFinite(P.x) || !Number.isFinite(P.y)) return;
  const ux = d.x / norm;
  const uy = d.y / norm;
  const half = LONG_LINE * norm;
  let lo = -half;
  let hi = half;
  for (const [p, u, size] of [[P.x, ux, w], [P.y, uy, h]]) {
    if (Math.abs(u) < 1e-12) {
      if (p < -LINE_MARGIN || p > size + LINE_MARGIN) return;
      continue;
    }
    const a = (-LINE_MARGIN - p) / u;
    const b = (size + LINE_MARGIN - p) / u;
    lo = Math.max(lo, Math.min(a, b));
    hi = Math.min(hi, Math.max(a, b));
  }
  if (lo > hi) return;
  const from = -half + period * Math.floor((lo + half) / period);
  const to = Math.min(half, hi + period);
  ctx.moveTo(P.x + ux * from, P.y + uy * from);
  ctx.lineTo(P.x + ux * to, P.y + uy * to);
}

/** Pole beams (and pole markers for point poles). */
export function drawPoleItems(ctx, F, T) {
  const { w, h, opts } = F;
  for (const it of F.items) {
    const picked = F.poleSel.has(it.id);
    const visible = (opts.beams && it.showBeam) || picked;
    if (!visible) continue;
    const P = T(it.point);
    ctx.strokeStyle = picked ? C.select : C.beam;
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = picked ? 2.5 : 1.5;
    if (it.kind === 'line') {
      ctx.setLineDash([8, 5]);
      ctx.beginPath();
      traceLongLine(ctx, P, sDir(it.dir), w, h, 13);
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      ctx.beginPath();
      ctx.arc(P.x, P.y, 10, 0, Math.PI * 2);
      ctx.stroke();
    }
    if (opts.poleLabels) labelText(ctx, it.label, P.x + 12, P.y - 10, { color: ctx.strokeStyle, font: 'bold 12px system-ui, sans-serif' });
  }
}

/** Kennedy auxiliary lines (thin dashed), under the supports. */
export function drawKennedyLines(ctx, F, T) {
  const { w, h } = F;
    ctx.strokeStyle = C.kennedy;
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 4]);
    for (const line of kennedyLines(F.poles)) {
      ctx.beginPath();
      traceLongLine(ctx, T(line.p), sDir(line.d), w, h, 6);
      ctx.stroke();
    }
    ctx.setLineDash([]);
}

/** Intersections of selected beams. */
export function drawIntersections(ctx, F, T) {
  const { w, h, opts } = F;
    F.intersections.forEach((x, k) => {
      const label = `X${k + 1}` + (opts.poleLabels && x.pair ? ` ${poleName(...x.pair, x.result.kind === 'parallel')}` : '');
      if (x.result.kind === 'point') {
        const P = T(x.result.point);
        ctx.fillStyle = C.inter;
        ctx.beginPath();
        ctx.arc(P.x, P.y, 5, 0, Math.PI * 2);
        ctx.fill();
        labelText(ctx, label, P.x + 8, P.y - 8, { color: C.inter, font: 'bold 12px system-ui, sans-serif' });
      } else if (x.result.kind === 'parallel') {
        drawInfinity(ctx, w, h, T(x.anchor), sDir(x.result.direction), opts.poleLabels && x.pair ? label : `${label} ∞`, C.inter);
      }
    });
}

/** Poles derived with Kennedy's theorem (skipped where a picked intersection already marks the point). */
export function drawDerivedPoles(ctx, F, T) {
  const { w, h, opts } = F;
    for (const pole of derivedPoles(F.poles)) {
      const label = derivedLabel(pole);
      if (pole.kind === 'point') {
        const covered = F.intersections.some((x) => x.result.kind === 'point' && len(sub(x.result.point, pole.point)) < 1e-6);
        if (covered) continue;
        const P = T(pole.point);
        ctx.strokeStyle = ctx.fillStyle = C.kennedy;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(P.x, P.y, 5, 0, Math.PI * 2);
        ctx.stroke();
        if (opts.poleLabels) labelText(ctx, label, P.x + 8, P.y - 8, { color: C.kennedy, font: 'bold 12px system-ui, sans-serif' });
      } else {
        drawInfinity(ctx, w, h, T(pole.via[0].p), sDir(pole.direction), opts.poleLabels ? label : '', C.kennedy);
      }
    }
}

/** Marks a point at infinity: an arrow to the canvas edge along `dir` (screen space) with a label. */
function drawInfinity(ctx, w, h, anchor, dir, label, color) {
  const m = 30;
  const edge = clipRayToRect(anchor, dir, { x0: m, y0: m, x1: w - m, y1: h - m });
  ctx.strokeStyle = ctx.fillStyle = color;
  ctx.lineWidth = 2;
  arrow(ctx, { x: edge.x - dir.x * 34, y: edge.y - dir.y * 34 }, edge);
  if (label) {
    labelText(ctx, label, edge.x - dir.y * 14 - dir.x * 10, edge.y + dir.x * 14 - dir.y * 10, { color, font: 'bold 13px system-ui, sans-serif', align: 'center' });
  }
}
