// Analysis panel: status line, DOF / mobility text, the mode selector, the picked poles and the Kennedy-derived poles.

import * as K from '../../core/kinematics.js';
import { derivedPoles, poleName } from '../../core/kennedy.js';
import { byId, setHtml } from '../dom.js';
import { fmt, esc } from '../format.js';

const itemName = (it) => `${it.label} ${it.kind === 'line' ? 'beam' : 'pole'}`;

/** Viewer: one checkbox per visible pole beam / pole marker (the editor picks them with the Pick poles tool). */
function pickList(state) {
  if (!state.items.length) return '<p class="hint">No pole beams or poles are shown.</p>';
  return '<div class="picks">' + state.items.map((it) =>
    `<label class="chk"><input type="checkbox" data-pick="${it.id}"${state.poleSel.has(it.id) ? ' checked' : ''}> ${esc(itemName(it))}</label>`).join('') + '</div>';
}

/** Descriptions of the intersections of the picked pole items, as list items. */
function intersectionItems(state) {
  const itemById = new Map(state.items.map((it) => [it.id, it]));
  const itemLabel = (id) => itemById.get(id)?.label ?? `#${id}`;
  return state.intersections.map((x, k) => {
    const r = x.result;
    const what = r.kind === 'point'
      ? `(${fmt(r.point.x)}, ${fmt(r.point.y)})`
      : r.kind === 'parallel'
        ? `∞, direction (${fmt(r.direction.x)}, ${fmt(r.direction.y)})`
        : 'coincident beams (no single point)';
    return `<li>X${k + 1} = ${itemLabel(x.a)} ∩ ${itemLabel(x.b)}: ${what}</li>`;
  }).join('');
}

/**
 * Wires the pole-picking controls of the panel.
 *  - state: the app state (poleSel is edited here, items are read).
 *  - viewer: true shows a checkbox list of poles instead of the "Pick poles" hint.
 *  - togglePole(id): toggles one pole item (the app refreshes and redraws).
 *  - onPicksChanged(): called after "all" / "none" changed state.poleSel.
 * Returns { update(state) }, which redraws the panel text for the current analysis.
 */
export function mountAnalysis({ state, viewer, togglePole, onPicksChanged }) {
  byId('btn-sel-all').addEventListener('click', () => {
    state.items.forEach((it) => state.poleSel.add(it.id));
    onPicksChanged();
  });
  byId('btn-sel-none').addEventListener('click', () => {
    state.poleSel.clear();
    onPicksChanged();
  });
  byId('poles-list').addEventListener('change', (e) => {
    if (e.target.dataset.pick) togglePole(Number(e.target.dataset.pick));
  });

  return {
    update(current) {
      const a = current.analysis;
      const d = K.describe(a);
      byId('status').textContent = d.short;
      const unattached = current.scene.supports.filter((s) => !current.sys.supportInfo.get(s.id).active).map((s) => `${s.type} at (${fmt(s.p.x)}, ${fmt(s.p.y)})`);
      setHtml(byId('analysis'),
        `<div class="big">${esc(d.short)}</div>` +
        `Bodies n = ${a.n}, constraint equations = ${a.eq}<br>` +
        `Grübler DOF 3n − eq = ${a.grubler}<br>` +
        `Instantaneous mobility = ${a.nullity}, redundant equations = ${a.redundant}<br>` +
        `<span class="hint">${esc(d.long)}</span>` +
        (unattached.length ? `<br><span class="hint">Not attached to a body (ignored): ${unattached.join(', ')}</span>` : ''));

      const modeSel = byId('opt-mode');
      // "Mode N" is slot N of the driver selection (see sim/drivers.js), named after its driver.
      const modeTexts = Array.from({ length: Math.max(1, a.nullity) }, (_, k) => (current.modeLabels[k] ? `Mode ${k + 1}: ${current.modeLabels[k]}` : `Mode ${k + 1}`));
      if (modeSel.dataset.sig !== modeTexts.join('|')) {
        modeSel.dataset.sig = modeTexts.join('|');
        modeSel.replaceChildren(...modeTexts.map((text, k) => new Option(text, String(k))));
      }
      modeSel.value = String(current.opts.mode);
      modeSel.disabled = a.nullity === 0;

      const picked = current.items.filter((it) => current.poleSel.has(it.id)).map(itemName);
      let html = viewer ? pickList(current) : picked.length ? `<p>Picked: ${picked.join(', ')}</p>` : '<p class="hint">Use the "Pick poles" tool or the checkbox on a support.</p>';
      if (current.intersections.length) html += '<ul>' + intersectionItems(current) + '</ul>';
      const derived = derivedPoles(current.poles);
      if (derived.length) {
        html += '<p>Derived with Kennedy’s theorem:</p><ul>' + derived.map((p) => (p.kind === 'point'
          ? `<li>${poleName(...p.pair)} = (${fmt(p.point.x)}, ${fmt(p.point.y)})</li>`
          : `<li>${poleName(...p.pair, true)}, direction (${fmt(p.direction.x)}, ${fmt(p.direction.y)})</li>`)).join('') + '</ul>';
      }
      setHtml(byId('poles-list'), html);
    },
  };
}
