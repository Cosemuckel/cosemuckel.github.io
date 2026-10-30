// Number and text formatting for the panels. DOM-free.

/** A coordinate / length with up to 3 decimals, trailing zeros removed, tiny values shown as 0. */
export const fmt = (v) => (Math.abs(v) < 5e-4 ? 0 : v).toFixed(3).replace(/\.?0+$/, '');

/** A motion value with 2 decimals, tiny values shown as 0. */
export const fmtT = (v) => (Math.abs(v) < 5e-3 ? 0 : v).toFixed(2);

/** Escapes text for use inside an HTML string. */
export const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]);
