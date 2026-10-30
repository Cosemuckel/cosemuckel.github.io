// Display options panel: the `data-opt` checkboxes / sliders and the mode selector, bound to state.opts.

import { byId, $$ } from '../dom.js';

/**
 * Binds every `[data-opt]` input to `state.opts[key]` and the mode selector to `state.opts.mode`.
 *  - state: the app state.
 *  - draw(): redraw after a display option changed.
 *  - scheduleHashUpdate(): the option is part of shared links.
 *  - refresh(): re-derive everything after the displayed mode changed.
 * Returns { sync() }, which writes the current values of state.opts back into the inputs (after a setup was loaded).
 */
export function mountOptions({ state, draw, scheduleHashUpdate, refresh }) {
  const sync = () => {
    for (const input of $$('[data-opt]')) {
      const key = input.dataset.opt;
      if (input.type === 'checkbox') input.checked = state.opts[key];
      else input.value = state.opts[key];
    }
    byId('scale-val').textContent = state.opts.scale.toFixed(2);
  };
  sync();
  for (const input of $$('[data-opt]')) {
    const key = input.dataset.opt;
    input.addEventListener('input', () => {
      state.opts[key] = input.type === 'checkbox' ? input.checked : Number(input.value);
      if (key === 'scale') byId('scale-val').textContent = state.opts.scale.toFixed(2);
      draw();
      scheduleHashUpdate();
    });
  }
  byId('opt-mode').addEventListener('change', (e) => {
    state.opts.mode = Number(e.target.value);
    refresh();
  });
  return { sync };
}
