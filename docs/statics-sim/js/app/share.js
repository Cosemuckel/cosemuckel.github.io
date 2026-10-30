// Share links: the whole setup lives in the URL hash (`s=`, see io/serialize.js), so the address bar is always a
// shareable link. This module owns the debounced address-bar writer, the "Copy link" / "Copy iframe code" actions and
// the message for a link that could not be read. App code (uses history, clipboard and window.location).

import * as M from '../core/model.js';
import { encodeSetup, hashWithSetup, VIEW_FLAGS } from '../io/serialize.js';
import { parseHash } from '../io/options.js';
import { encodeView, hashWithView } from '../io/viewlink.js';
import { HASH_DEBOUNCE_MS } from '../constants.js';
import { reportWarning, describeError } from './errors.js';

const LONG_LINK_CHARS = 8000; // beyond this some services truncate links
const LOAD_ERROR_MS = 12000; // how long the "could not read the link" notice stays

/** The shareable part of the app state as a serialize.js setup (the scene is copied, not referenced). */
export function captureSetup(state) {
  return {
    scene: M.cloneScene(state.scene),
    flags: Object.fromEntries(VIEW_FLAGS.map((name) => [name, !!state.opts[name]])),
    scale: state.opts.scale,
    mode: state.opts.mode,
    picks: [...state.poleSel],
    locked: state.sim.locked.map(Boolean),
    ...(state.sim.selection && state.sim.selection.length ? { drivers: state.sim.selection.slice() } : {}),
  };
}

/** Clipboard write with the legacy fallback; last resort is a prompt the user can copy from. Resolves to whether the text was copied. */
export async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);
    return true;
  } catch (err) {
    // Clipboard API missing or denied (insecure origin, iframe policy): try the legacy path below.
    reportWarning('Clipboard API unavailable, using the legacy copy', err);
  }
  const area = Object.assign(document.createElement('textarea'), { value: text });
  area.style.cssText = 'position:fixed;left:-1000px;top:0;opacity:0';
  document.body.append(area);
  area.select();
  let ok = false;
  try {
    ok = document.execCommand('copy');
  } catch (err) {
    reportWarning('Legacy copy failed', err);
  }
  area.remove();
  if (!ok) window.prompt('Copy this text:', text);
  return ok;
}

/** Status text for a link whose setup cannot be loaded: the cause (ended with a full stop) followed by the fallback taken. */
export function loadErrorMessage(err) {
  const cause = describeError(err);
  return `Could not load the setup from the link: ${cause}${/[.!?]$/.test(cause) ? '' : '.'} Showing the default example instead.`;
}

/**
 * Creates the address-bar sync for `state` (the app state, read when a link is written).
 * `viewer` pages never rewrite the address bar; `message(text, ms)` shows a status line.
 *
 * Returns:
 *  - scheduleHashUpdate(): the scene or pose may have changed; re-encode the setup at the next (debounced) write.
 *  - scheduleViewUpdate(): same debounce for a view-only change (wheel / pan / Fit); only the `cam` key is patched.
 *  - start(): call once after the initial setup is loaded; later changes rewrite the address bar relative to that baseline.
 *  - copyShare(embed): copies the link, or the iframe code for the read-only viewer when `embed`.
 *  - reportLoadError(err): tells the user that the link could not be read and the default example is shown instead.
 */
export function createShare({ state, viewer, message }) {
  const link = { timer: 0, seq: 0, last: '', lastView: null, dirty: false, live: false };

  /** `cam` text for the current zoom/pan: null while the view is "fit" (or cannot be written). */
  const currentViewText = () => (state.viewCustom ? encodeView(state.view, state.size) : null);

  function scheduleHashUpdate() {
    link.dirty = true;
    scheduleViewUpdate();
  }

  function scheduleViewUpdate() {
    if (!link.live) return;
    clearTimeout(link.timer);
    link.timer = setTimeout(writeHash, HASH_DEBOUNCE_MS);
  }

  /** Writes the current setup into the address bar (history.replaceState, no new history entries). */
  async function writeHash() {
    const seq = ++link.seq;
    let encoded = link.last;
    if (link.dirty || !encoded) {
      encoded = await encodeSetup(captureSetup(state));
      if (seq !== link.seq) return; // superseded by a later change
      link.dirty = false;
    }
    const view = currentViewText();
    if (encoded === link.last && view === link.lastView) return; // nothing changed since the last write
    link.last = encoded;
    link.lastView = view;
    try {
      history.replaceState(null, '', '#' + hashWithView(hashWithSetup(parseHash(window.location.hash), encoded), view));
    } catch (err) {
      link.live = false; // e.g. sandboxed frame: keep working, only the address bar stays untouched
      reportWarning('Could not update the address bar', err);
    }
  }

  /** Absolute URL of the current setup; `embed` makes it the read-only viewer used inside an iframe. */
  async function setupUrl({ embed }) {
    let hash = parseHash(window.location.hash);
    if (embed) {
      for (const k of ['toolbar', 'panel', 'edit', 'view', 'embed', 'cam']) hash.delete(k);
      hash = new URLSearchParams([['embed', '1'], ...hash]); // embed first: survives truncation / bad escaping of what follows
    }
    return window.location.href.split('#')[0] + '#' + hashWithView(hashWithSetup(hash, await encodeSetup(captureSetup(state))), currentViewText());
  }

  async function copyShare(embed) {
    const url = await setupUrl({ embed });
    const text = embed
      ? `<iframe src="${url.replace(/&/g, '&amp;')}" style="width:100%;height:480px;border:1px solid #ccc" title="Statics simulator" loading="lazy"></iframe>`
      : url;
    const ok = await copyText(text);
    const long = url.length > LONG_LINK_CHARS ? ` Note: it is ${url.length} characters long; some services truncate very long links.` : '';
    message((ok ? (embed ? 'Iframe code copied.' : 'Link copied.') : 'Copy it from the dialog.') + long, 4000);
  }

  function start() {
    link.live = !viewer; // a viewer never rewrites the address bar
    link.lastView = currentViewText();
    encodeSetup(captureSetup(state)).then((encoded) => { link.last = encoded; });
  }

  function reportLoadError(err) {
    message(loadErrorMessage(err), LOAD_ERROR_MS);
  }

  return { scheduleHashUpdate, scheduleViewUpdate, start, copyShare, reportLoadError };
}
