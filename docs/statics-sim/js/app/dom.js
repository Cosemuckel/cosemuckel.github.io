// DOM lookup and small element builders shared by the app modules. App code only (core/sim/io stay DOM-free).

/** The element with this id, or null (for the few elements that are absent in the viewer). */
export function maybeById(id) {
  return document.getElementById(id);
}

/** The element with this id; throws a clear error when the page does not have it. */
export function byId(id) {
  const el = document.getElementById(id);
  if (!el) throw new Error(`Required element #${id} is missing from the page`);
  return el;
}

/** The first element matching `selector` under `root`; throws when there is none. */
export function $(selector, root = document) {
  const el = root.querySelector(selector);
  if (!el) throw new Error(`Required element '${selector}' is missing from the page`);
  return el;
}

/** Every element matching `selector` under `root`, as an array. */
export function $$(selector, root = document) {
  return [...root.querySelectorAll(selector)];
}

/** Creates `<tag>` with `props` assigned as properties and `children` appended. */
export function mk(tag, props = {}, ...children) {
  const el = Object.assign(document.createElement(tag), props);
  el.append(...children);
  return el;
}

/** A labelled checkbox; `onChange(checked)` runs on change. */
export function checkbox(label, checked, onChange) {
  const input = mk('input', { type: 'checkbox', checked });
  input.addEventListener('change', () => onChange(input.checked));
  return mk('label', { className: 'chk' }, input, label);
}

/** A labelled number input; `onChange(value)` runs on change, only for a finite number (an empty or invalid entry is ignored). */
export function numberField(label, value, step, onChange) {
  const input = mk('input', { type: 'number', value: String(+Number(value).toFixed(4)), step: String(step) });
  input.addEventListener('change', () => {
    if (input.value !== '' && Number.isFinite(Number(input.value))) onChange(Number(input.value));
  });
  return mk('label', { className: 'field' }, label, input);
}

const lastHtml = new WeakMap();
/** Sets `el.innerHTML` only when it differs from what this function wrote last (avoids resetting selection / focus while playing). */
export function setHtml(el, html) {
  if (lastHtml.get(el) === html) return;
  lastHtml.set(el, html);
  el.innerHTML = html;
}

/**
 * A status-line writer for the element `el`: `show(text, ms = 3500)` displays `text` and hides it after `ms` (0 = stays);
 * empty text hides the line at once. A new message replaces the pending hide of the previous one.
 */
export function createNotice(el) {
  let timer = 0;
  function show(text, ms = 3500) {
    el.textContent = text;
    el.style.display = text ? 'block' : 'none';
    clearTimeout(timer);
    if (text && ms) timer = setTimeout(() => show(''), ms);
  }
  return show;
}
