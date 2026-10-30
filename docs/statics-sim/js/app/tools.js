// Pointer-driven editing tools: hit testing, select/drag, line chains, supports, poles, delete, keyboard shortcuts and the
// toolbar. All maths lives in core/; the pure helpers exported here take the app state explicitly so they run without a DOM.

import * as G from '../core/geometry.js';
import * as M from '../core/model.js';
import * as K from '../core/kinematics.js';
import { screenToWorld, worldToScreen, snapToGrid, zoomAt, panBy, eventPosition } from './view.js';
import { byId } from './dom.js';

export const PICK_PX = 10; // hit radius for supports / poles (screen px)
export const LINE_PX = 7; // hit distance for bodies and beams (screen px)
export const CLICK_PX = 4; // pointer travel below this is a click, not a drag

// ------------------------------------------------------------------ hit tests (pure)

const currentPoints = (state, i) => state.sys.bodies[i].pts.map((p) => K.worldPoint(state.q, i, p));

/** Topmost body index under world point w, or -1. */
export function hitBody(state, w) {
  const tol = LINE_PX / state.view.scale;
  for (let i = state.scene.bodies.length - 1; i >= 0; i--) {
    const b = state.scene.bodies[i];
    if (M.pointHitsShape(b.kind, currentPoints(state, i), w, tol, b.segs)) return i;
  }
  return -1;
}

/** Topmost support whose current position is within PICK_PX of screen point sp, or null. */
export function hitSupport(state, sp) {
  for (let i = state.scene.supports.length - 1; i >= 0; i--) {
    const s = state.scene.supports[i];
    const at = worldToScreen(state.view, state.size, K.supportPoint(state.sys, state.q, s));
    if (G.dist(at, sp) <= PICK_PX) return s;
  }
  return null;
}

/** Pole item (visible beam or pole marker) under screen point sp. */
export function hitPoleItem(state, sp) {
  for (const it of state.items) {
    const P = worldToScreen(state.view, state.size, it.point);
    if (it.kind === 'point') {
      if (G.dist(P, sp) <= PICK_PX + 2) return it;
    } else {
      const d = { x: it.dir.x, y: -it.dir.y };
      if (Math.abs(G.cross(d, G.sub(sp, P))) <= LINE_PX) return it;
    }
  }
  return null;
}

// ------------------------------------------------------------ chain logic (pure)

/**
 * Start point for "Add segment" at world point w: a node of the clicked rod, else a point on it.
 * Returns { id, p } or { error } (a user-facing reason why nothing can be attached).
 */
export function attachStart(state, w, snap) {
  if (state.q.some((v) => v !== 0)) return { error: 'Reset the simulated pose before adding a segment.' };
  const bi = hitBody(state, w);
  if (bi < 0) return { error: 'Click a line body to add a segment to it.' };
  const body = state.scene.bodies[bi];
  const tol = LINE_PX / state.view.scale;
  const node = body.pts.find((p) => G.dist(p, w) <= tol);
  const snapped = snap(w);
  const onBody = G.dist(M.nearestPointOnBody(body, snapped), snapped) < 1e-6;
  return { id: body.id, p: node ? { ...node } : onBody ? snapped : M.nearestPointOnBody(body, w) };
}

/** A click while a chain is open finishes it when it repeats the last point (double-click), in world or screen space. */
export function chainClickFinishes(lastWorld, nextWorld, lastScreen, clickScreen) {
  return G.dist(nextWorld, lastWorld) < 1e-9 || G.dist(lastScreen, clickScreen) < CLICK_PX;
}

/**
 * Creates a body from the chain, or (chain.target) adds the points to that body, merging a rod the last point ends on.
 * Returns the id of the created/extended body, or null when the scene rejected the geometry.
 */
export function commitChainToScene(scene, chain, tol) {
  const pts = chain.pts.slice();
  if (chain.target != null) {
    const end = pts[pts.length - 1];
    const other = scene.bodies.find((b) => b.id !== chain.target && b.kind === 'rod' && M.pointHitsShape(b.kind, b.pts, end, tol, b.segs));
    if (other) pts[pts.length - 1] = M.nearestPointOnBody(other, end);
    if (!M.extendBody(scene, chain.target, pts)) return null;
    if (other) M.mergeBodies(scene, chain.target, other.id);
    return chain.target;
  }
  const body = M.addPolyline(scene, pts);
  return body ? body.id : null;
}

// ---------------------------------------------------------------------- mounting

/**
 * Attach pointer navigation (wheel zoom, panning) and - unless viewer - the editing tools, keyboard shortcuts and toolbar.
 * Returns { setTool }.
 *
 * ctx: { state, store, canvas, viewer, zoomEnabled, message(text), edited(), refresh(), draw(), renderProps(),
 *        togglePole(id), dropTrajectory(), viewMoved() }
 */
export function mountTools(ctx) {
  const { state: S, store, canvas, viewer, zoomEnabled, message, edited, refresh, draw, renderProps, togglePole, dropTrajectory, viewMoved } = ctx;

  const toWorld = (sp) => screenToWorld(S.view, S.size, sp);
  const toScreen = (p) => worldToScreen(S.view, S.size, p);
  const snap = (p) => (S.opts.snap ? snapToGrid(S.view, p) : p);
  const eventPos = (e) => eventPosition(canvas, e);

  function selectItem(sel) {
    S.sel = sel;
    renderProps();
    draw();
  }

  function setTool(tool) {
    if (S.chain && tool !== S.tool) finishChain();
    store.set({ tool });
  }

  // -- navigation (also in the viewer)

  canvas.addEventListener('contextmenu', (e) => e.preventDefault());
  canvas.addEventListener('wheel', (e) => {
    if (viewer && !zoomEnabled) return; // let the embedding page scroll
    e.preventDefault();
    S.view = zoomAt(S.view, S.size, eventPos(e), e.deltaY);
    viewMoved();
    draw();
  }, { passive: false });

  if (viewer) {
    // Read-only: the only pointer action is panning the view.
    canvas.addEventListener('pointerdown', (e) => {
      if (S.drag || !zoomEnabled) return;
      canvas.setPointerCapture(e.pointerId);
      S.drag = { kind: 'pan', last: eventPos(e) };
    });
    canvas.addEventListener('pointermove', (e) => {
      const d = S.drag;
      if (!d) return;
      const sp = eventPos(e);
      S.view = panBy(S.view, sp.x - d.last.x, sp.y - d.last.y);
      d.last = sp;
      viewMoved();
      draw();
    });
    const stopPan = () => {
      S.drag = null;
      S.preview = null;
      draw();
    };
    canvas.addEventListener('pointerup', stopPan);
    canvas.addEventListener('pointercancel', stopPan);
    return { setTool };
  }

  // -- editing pointer handling

  canvas.addEventListener('pointerdown', (e) => {
    if (S.drag) return; // ignore additional touches
    const sp = eventPos(e);
    canvas.setPointerCapture(e.pointerId);
    const w = toWorld(sp);
    if (e.button === 1 || e.button === 2) {
      S.drag = { kind: 'pan', last: sp };
      return;
    }
    switch (S.tool) {
      case 'select': return downSelect(sp, w);
      case 'line': case 'addseg': downLine(sp, w); break;
      case 'pin': case 'roller': case 'joint': placeSupport(S.tool, snap(w)); break;
      case 'poles': {
        const it = hitPoleItem(S, sp);
        if (it) togglePole(it.id);
        break;
      }
      case 'delete': deleteAt(sp, w); break;
    }
    draw();
  });

  canvas.addEventListener('pointermove', (e) => {
    const d = S.drag;
    if (!d) {
      if (S.chain) {
        S.preview = chainPreview(snap(toWorld(eventPos(e))));
        draw();
      }
      return;
    }
    const sp = eventPos(e);
    const w = toWorld(sp);
    switch (d.kind) {
      case 'chainClick': return;
      case 'pan':
        S.view = panBy(S.view, sp.x - d.last.x, sp.y - d.last.y);
        d.last = sp;
        viewMoved();
        break;
      case 'line': S.preview = { kind: 'line', a: d.a, b: snap(w) }; break;
      case 'moveSupport': {
        const s = S.scene.supports.find((x) => x.id === d.id);
        s.p = snap(w);
        edited();
        return;
      }
      case 'moveBody': moveBody(d, snap(w)); return;
      case 'simDrag':
        S.q = K.dragTo(S.sys, S.q, d.body, d.pRef, w);
        dropTrajectory();
        refresh();
        return;
    }
    draw();
  });

  function endDrag(e) {
    const d = S.drag;
    if (!d) return;
    S.drag = null;
    S.preview = S.chain ? chainPreview(snap(toWorld(eventPos(e)))) : null;
    if (d.kind === 'line') {
      const sp = eventPos(e);
      const b = snap(toWorld(sp));
      if (G.dist(sp, d.sp) < CLICK_PX) {
        // A click, not a drag: start a multi-segment chain from the press point.
        S.chain = { target: d.target, pts: [d.a] };
        S.preview = chainPreview(d.a);
        message('Click further points; double-click, Enter or Esc finishes the body.');
      } else if (G.dist(d.a, b) > 1e-6) {
        commitChain({ target: d.target, pts: [d.a, b] });
      }
    } else if (d.kind === 'chainClick') {
      clickChain(eventPos(e));
    }
    draw();
  }
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', () => {
    S.drag = null;
    S.preview = null;
    draw();
  });

  // -- line / add-segment tools: drag = one segment; click = start a chain of connected segments

  function downLine(sp, w) {
    if (S.chain) {
      S.drag = { kind: 'chainClick' };
      return;
    }
    let a = snap(w);
    let target = null;
    if (S.tool === 'addseg') {
      const start = attachStart(S, w, snap);
      if (start.error) return message(start.error);
      a = start.p;
      target = start.id;
      selectItem({ kind: 'body', id: target });
    }
    S.drag = { kind: 'line', a, sp, target };
    S.preview = { kind: 'line', a, b: a };
  }

  const chainPreview = (cursor) => ({ kind: 'chain', pts: [...S.chain.pts, cursor] });

  function clickChain(sp) {
    const last = S.chain.pts[S.chain.pts.length - 1];
    const next = snap(toWorld(sp));
    if (chainClickFinishes(last, next, toScreen(last), sp)) return finishChain();
    S.chain.pts.push(next);
    S.preview = chainPreview(next);
  }

  function finishChain() {
    const chain = S.chain;
    S.chain = null;
    S.preview = null;
    if (chain && chain.pts.length >= 2) commitChain(chain);
    draw();
  }

  function commitChain(chain) {
    const id = commitChainToScene(S.scene, chain, LINE_PX / S.view.scale);
    if (id == null) return;
    S.sel = { kind: 'body', id };
    edited();
  }

  // -- select / move

  function downSelect(sp, w) {
    const sup = hitSupport(S, sp);
    const bi = hitBody(S, w);
    if (S.sim.on && bi >= 0 && !sup) {
      S.drag = { kind: 'simDrag', body: bi, pRef: K.bodyPoint(S.q, bi, w) };
      selectItem({ kind: 'body', id: S.scene.bodies[bi].id });
      return;
    }
    if (sup) {
      selectItem({ kind: 'support', id: sup.id });
      if (!S.sim.on) S.drag = { kind: 'moveSupport', id: sup.id };
    } else if (bi >= 0) {
      const body = S.scene.bodies[bi];
      selectItem({ kind: 'body', id: body.id });
      if (!S.sim.on) {
        // Carry pins/rollers that currently sit on this body (their topmost body).
        const carried = S.scene.supports.filter((s) => {
          const info = S.sys.supportInfo.get(s.id);
          return s.type !== 'joint' && info.active && info.body === bi;
        });
        S.drag = {
          kind: 'moveBody', id: body.id, start: snap(w),
          pts: body.pts.map((p) => ({ ...p })),
          supports: carried.map((s) => ({ id: s.id, p: { ...s.p } })),
        };
      }
    } else {
      selectItem(null);
      S.drag = { kind: 'pan', last: sp };
    }
    draw();
  }

  function moveBody(d, now) {
    const dx = now.x - d.start.x;
    const dy = now.y - d.start.y;
    const body = S.scene.bodies.find((b) => b.id === d.id);
    body.pts = d.pts.map((p) => ({ x: p.x + dx, y: p.y + dy }));
    for (const s of d.supports) {
      const target = S.scene.supports.find((x) => x.id === s.id);
      if (target) target.p = { x: s.p.x + dx, y: s.p.y + dy };
    }
    edited();
  }

  function placeSupport(type, p) {
    const n = M.bodiesAtPoint(S.scene, p).length;
    if (type === 'joint' && n < 2) return message('A joint needs at least two bodies at this point.');
    if (type !== 'joint' && n < 1) return message('Place supports on a body.');
    const sup = M.addSupport(S.scene, type, p);
    S.sel = { kind: 'support', id: sup.id };
    edited();
  }

  function deleteAt(sp, w) {
    const sup = hitSupport(S, sp);
    if (sup) {
      M.removeSupport(S.scene, sup.id);
    } else {
      const bi = hitBody(S, w);
      if (bi < 0) return;
      M.removeBody(S.scene, S.scene.bodies[bi].id);
    }
    edited();
  }

  // -- keyboard

  window.addEventListener('keydown', (e) => {
    if (/^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
    if ((e.key === 'Escape' || e.key === 'Enter') && S.chain) {
      finishChain(); // commits when it has a segment; a second Esc then deselects as usual
    } else if (e.key === 'Escape') {
      S.drag = null;
      S.preview = null;
      setTool('select');
      selectItem(null);
    } else if ((e.key === 'Delete' || e.key === 'Backspace') && S.sel) {
      if (S.sel.kind === 'body') M.removeBody(S.scene, S.sel.id);
      else M.removeSupport(S.scene, S.sel.id);
      S.sel = null;
      edited();
    }
  });

  // -- toolbar: buttons and cursor follow the active tool (store listener)

  store.subscribe((state, patch) => {
    if (!('tool' in patch)) return;
    for (const b of document.querySelectorAll('#tools button')) {
      const on = b.dataset.tool === state.tool;
      b.classList.toggle('active', on);
      b.setAttribute('aria-pressed', String(on));
    }
    canvas.style.cursor = state.tool === 'select' ? 'default' : 'crosshair';
  });
  byId('tools').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-tool]');
    if (b) setTool(b.dataset.tool);
  });

  return { setTool };
}
