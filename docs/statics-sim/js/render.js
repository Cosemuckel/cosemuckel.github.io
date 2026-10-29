// Canvas rendering. Reads a "frame" object, draws, keeps no state of its own.
//
// frame = {
//   w, h, dpr, view: {cx, cy, scale},        canvas size (CSS px) and camera (world centre, px per unit)
//   scene, sys, q, dq, items,                 model, constraint system, pose, unit mode (or null), pole items
//   intersections, poles,                     result of selectedIntersections(), derivePoles() map
//   opts, sel, poleSel, preview,              UI options / selection / drawing preview
// }

import { sub, add, mul, len, gridStep, clipRayToRect } from './geometry.js';
import { bodySegments, nearestPointOnBody } from './model.js';
import { worldPoint, pointVelocity, supportPoint, bodyMotion, poleOf } from './kinematics.js';
import { derivedPoles, kennedyLines, derivedLabel, poleName } from './kennedy.js';

const C = {
  bg: '#fbfcfd',
  grid: '#e3e8ee',
  axis: '#b7c0cc',
  body: '#2b5d9b',
  ghost: '#8d99a8',
  displaced: '#d9822b',
  beam: '#c0392b',
  select: '#e6a700',
  inter: '#1e8e3e',
  kennedy: '#2a8f8f',
  support: '#243447',
  text: '#243447',
};

export function render(ctx, F) {
  const { w, h, view, opts } = F;
  const T = (p) => ({ x: (p.x - view.cx) * view.scale + w / 2, y: h / 2 - (p.y - view.cy) * view.scale });
  const sDir = (d) => ({ x: d.x, y: -d.y }); // world direction -> screen direction

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

  // Bodies
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

  // Pole beams (and pole markers for point poles)
  for (const it of F.items) {
    const picked = F.poleSel.has(it.id);
    const visible = (opts.beams && it.showBeam) || picked;
    if (!visible) continue;
    const P = T(it.point);
    ctx.strokeStyle = picked ? C.select : C.beam;
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = picked ? 2.5 : 1.5;
    if (it.kind === 'line') {
      const d = sDir(it.dir);
      const L = 1e4;
      ctx.setLineDash([8, 5]);
      ctx.beginPath();
      ctx.moveTo(P.x - d.x * L, P.y - d.y * L);
      ctx.lineTo(P.x + d.x * L, P.y + d.y * L);
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      ctx.beginPath();
      ctx.arc(P.x, P.y, 10, 0, Math.PI * 2);
      ctx.stroke();
    }
    if (opts.poleLabels) labelText(ctx, it.label, P.x + 12, P.y - 10, { color: ctx.strokeStyle, font: 'bold 12px system-ui, sans-serif' });
  }

  // Kennedy auxiliary lines (thin dashed), under the supports
  if (opts.inters && opts.kennedy) {
    ctx.strokeStyle = C.kennedy;
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 4]);
    for (const line of kennedyLines(F.poles)) {
      const P = T(line.p);
      const d = sDir(line.d);
      const L = 1e4;
      ctx.beginPath();
      ctx.moveTo(P.x - d.x * L, P.y - d.y * L);
      ctx.lineTo(P.x + d.x * L, P.y + d.y * L);
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  // Supports
  for (const sp of F.scene.supports) {
    const info = F.sys.supportInfo.get(sp.id);
    const P = T(supportPoint(F.sys, q, sp));
    const selected = F.sel && F.sel.kind === 'support' && F.sel.id === sp.id;
    drawSupport(ctx, sp, P, info && info.active, selected);
  }

  // Intersections of selected beams
  if (opts.inters) {
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

  // Poles derived with Kennedy's theorem (skipped where a picked intersection already marks the point)
  if (opts.inters) {
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

  // Displacement vectors and kinematic body poles
  if (showDisp && opts.vectors) {
    ctx.strokeStyle = ctx.fillStyle = C.displaced;
    ctx.lineWidth = 1.5;
    cur.forEach((pts, i) => pts.forEach((P, k) => arrow(ctx, T(P), T(disp[i][k]))));
  }
  if (F.dq && opts.bodyPoles) {
    F.sys.bodies.forEach((b, i) => {
      const pole = poleOf(bodyMotion(q, F.dq, i));
      if (!pole) return;
      const label = opts.poleLabels ? poleName(0, i + 1, pole.kind === 'infinity') : `P${i + 1}`;
      if (pole.kind === 'point') {
        const P = T(pole.point);
        ctx.fillStyle = '#7b3fb0';
        ctx.beginPath();
        ctx.arc(P.x, P.y, 5, 0, Math.PI * 2);
        ctx.fill();
        labelText(ctx, label, P.x + 8, P.y - 8, { color: '#7b3fb0', font: 'bold 12px system-ui, sans-serif' });
      } else {
        drawInfinity(ctx, w, h, T(centroid(cur[i])), sDir(pole.direction), label, '#7b3fb0');
      }
    });
  }

  // Tool preview
  const pv = F.preview;
  if (pv) {
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
}

function centroid(pts) {
  const sum = pts.reduce((a, p) => add(a, p), { x: 0, y: 0 });
  return mul(sum, 1 / pts.length);
}

function drawGrid(ctx, F, T) {
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

function arrow(ctx, a, b) {
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

/** Text with a white halo so it stays readable over lines and bodies. */
function labelText(ctx, text, x, y, { color, font, align = 'left' }) {
  ctx.font = font;
  ctx.textAlign = align;
  ctx.lineJoin = 'round';
  ctx.lineWidth = 3.5;
  ctx.strokeStyle = 'rgba(255,255,255,0.92)';
  ctx.strokeText(text, x, y);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}
