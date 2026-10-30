import { startApp } from './app/ui.js';
import { decodeSetup } from './io/serialize.js';
import { reportError } from './app/errors.js';
import { readParams } from './io/options.js';

/**
 * Options come from the query string and/or the URL hash (e.g. "#embed=1&example=arch"),
 * so an embedding page can use either. The hash wins when both define a key.
 * Option names are case-insensitive; on/off options accept 1|true|yes|on (or a bare name) and 0|false|no|off.
 *   s=TEXT           whole setup (bodies, supports, display flags, ...) as written by the editor; wins over example
 *   embed=1          read-only viewer for iframes, no title bar (view=1: same viewer with the title bar)
 *   edit=1           with embed=1/view=1: keep the full editor
 *   zoom=0           viewer only: freeze the view (no wheel zoom, no panning)
 *   toolbar=0        hide the toolbar (editor only)
 *   panel=0|1        force the side panel closed / open (default: open when the window is wider than 720 px)
 *   example=NAME     mechanism (default) | chain | slidercrank | arch | frame
 *   displaced=1      start with the displaced-shape overlay on
 */
const params = readParams(window.location.search, window.location.hash);

// Reaching this point means ES modules work: drop the static "serve over HTTP" notice from index.html.
document.getElementById('boot-msg')?.remove();

const initial = {};
if (params.has('s')) {
  try {
    initial.setup = await decodeSetup(params.get('s'));
  } catch (err) {
    reportError('Could not read the setup in the URL', err);
    initial.error = err;
  }
}

startApp(params, initial);
