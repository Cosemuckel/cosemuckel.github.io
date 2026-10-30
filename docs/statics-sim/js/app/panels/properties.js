// Property panel of the selected body or support. Rebuilt on selection changes and edits, never during playback.
// The panel is absent in the viewer.

import * as G from '../../core/geometry.js';
import * as M from '../../core/model.js';
import { maybeById, mk, checkbox, numberField } from '../dom.js';
import { fmt } from '../format.js';
import { findSelected } from '../state.js';

const SUPPORT_NAMES = { pin: 'Pin support', roller: 'Roller support', joint: 'Joint' };

/**
 * Mounts the property panel.
 *  - state: the app state (scene, sel, sys, items, poleSel are read; the selected item is edited through the fields).
 *  - edited(): the scene geometry or supports changed (rebuilds analysis and redraws).
 *  - draw(), scheduleHashUpdate(): for display-only changes (they do not alter the mechanism).
 *  - togglePole(id): toggles a pole item in the pole selection.
 * Returns { render() }, which rebuilds the panel for the current selection.
 */
export function mountProperties({ state, edited, draw, scheduleHashUpdate, togglePole }) {
  function renderBody(box, item) {
    const p = item.pts;
    const num = state.scene.bodies.indexOf(item) + 1; // body number as drawn on the canvas
    const segs = M.bodySegments(item.kind, p, item.segs);
    const info = segs.length === 1
      ? `Rod ${num}: length ${fmt(G.dist(p[0], p[1]))}, angle ${fmt((Math.atan2(p[1].y - p[0].y, p[1].x - p[0].x) * 180) / Math.PI)}°`
      : `Line body ${num}: ${segs.length} segments, total length ${fmt(segs.reduce((sum, [i, j]) => sum + G.dist(p[i], p[j]), 0))}`;
    box.append(mk('p', {}, info));
    box.append(mk('button', { textContent: 'Delete body', onclick: () => { M.removeBody(state.scene, item.id); state.sel = null; edited(); } }));
  }

  function renderSupport(box, s) {
    const info = state.sys.supportInfo.get(s.id);
    box.append(mk('p', {}, SUPPORT_NAMES[s.type] + (info.active ? ` on body ${info.bodies.map((i) => i + 1).join(', ')}` : '')));
    if (!info.active) box.append(mk('p', { className: 'hint' }, 'Not attached to enough bodies - ignored by the analysis.'));
    box.append(numberField('x', s.p.x, 0.5, (v) => { s.p.x = v; edited(); }));
    box.append(numberField('y', s.p.y, 0.5, (v) => { s.p.y = v; edited(); }));
    if (s.type === 'roller') {
      box.append(numberField('Slide direction (°)', s.angleDeg, 5, (v) => { s.angleDeg = v; edited(); }));
      box.append(checkbox('Lock sliding (acts as pin)', s.locked, (v) => { s.locked = v; edited(); }));
    }
    if (s.type === 'pin') box.append(checkbox('Clamped (also blocks rotation)', s.clamped, (v) => { s.clamped = v; edited(); }));
    box.append(checkbox(s.type === 'roller' && !s.locked ? 'Show pole beam' : 'Show pole marker', s.showBeam, (v) => { s.showBeam = v; draw(); scheduleHashUpdate(); }));
    if (state.items.some((it) => it.id === s.id)) {
      box.append(checkbox('Pick for intersections', state.poleSel.has(s.id), () => togglePole(s.id)));
    } else if (s.type === 'pin' && s.clamped) {
      box.append(mk('p', { className: 'hint' }, 'A clamped pin has no pole (the body cannot move).'));
    }
    box.append(mk('button', { textContent: 'Delete support', onclick: () => { M.removeSupport(state.scene, s.id); state.sel = null; edited(); } }));
  }

  return {
    render() {
      const box = maybeById('props');
      if (!box) return; // viewer: the property panel is not part of the page
      box.innerHTML = '';
      const item = findSelected(state);
      if (!item) {
        box.append(mk('p', { className: 'hint' }, 'Nothing selected. Click a body or support with the Select tool.'));
      } else if (state.sel.kind === 'body') {
        renderBody(box, item);
      } else {
        renderSupport(box, item);
      }
    },
  };
}
