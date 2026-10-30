// The one place where failures become output: a visible status message plus a console entry that keeps the context
// and the original error (stack included). Nothing in the app swallows an error silently; code that recovers from a
// failure reports it through here and then carries on with its fallback.
//
// Expected errors (SetupError: a link that cannot be read) are the user's input being wrong, not a bug: they get an
// actionable message and a console warning. Everything else is logged with console.error.

import { SetupError } from '../io/serialize.js';
import { NOTICE_LONG_MS } from '../constants.js';

let showStatus = null;

/** Connects the visible status line: `show(text, durationMs)`. Before this is called errors only reach the console. */
export function setStatusSink(show) {
  showStatus = show;
}

/** User-facing text of an error: what failed and, for a bad link, what to do about it. */
export function describeError(err) {
  if (err instanceof SetupError) return `${err.message}. The link may be cut off or come from an incompatible version; copy it again.`;
  return err instanceof Error && err.message ? err.message : String(err);
}

/**
 * Reports a failure that stops `context` (a short noun phrase, e.g. "Motion unavailable"): shown in the status line
 * as "<context>: <description>" and logged with the original error.
 */
export function reportError(context, err, { durationMs = NOTICE_LONG_MS } = {}) {
  if (err instanceof SetupError) console.warn(`${context}:`, err);
  else console.error(`${context}:`, err);
  if (showStatus) showStatus(`${context}: ${describeError(err)}`, durationMs);
}

/** Reports a failure the app recovers from by itself (a fallback is used): logged only, the status line stays quiet. */
export function reportWarning(context, err) {
  console.warn(`${context}:`, err);
}
