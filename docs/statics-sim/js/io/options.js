// @ts-check
// URL options: parsing of the query string / hash into URLSearchParams and the typed readers on top of it (pure: no
// DOM, so the page script passes window.location in). Setup payloads (`s=`) live in serialize.js, the camera (`cam=`)
// in viewlink.js; this module only knows how option text is split, normalised and interpreted.

/** URL key of the framing, and the legacy key that used to carry it (`view=cx,cy,r`, see framingText). */
export const CAM_KEY = 'cam';
export const LEGACY_KEY = 'view';

/**
 * Option text may reach the page HTML-escaped ("&amp;") or percent-escaped ("%26") when an iframe snippet was
 * pasted into a context that does not decode it; neither character sequence can occur in a base64url setup or
 * in a plain option value, so both are read as the "&" separator. Without this, "#s=...&amp;embed=1" silently
 * lost its embed flag and showed the full editor.
 */
const normalizeOptionText = (text) => String(text).replace(/&amp;|%26/gi, '&');

/**
 * URLSearchParams with option keys lower-cased ("EMBED=1" is "embed=1"). Values are untouched: the `s` setup
 * payload is case-sensitive base64url, and a key that only differs from `s` by case is left alone rather than
 * being turned into the payload key.
 */
function parseOptions(text) {
  const params = new URLSearchParams();
  for (const [k, v] of new URLSearchParams(normalizeOptionText(text))) {
    const lower = k.toLowerCase();
    params.append(lower === 's' ? k : lower, v);
  }
  return params;
}

/** Parses "#a=1&b=2" / "#?a=1" into URLSearchParams (keys lower-cased, see parseOptions). */
export const parseHash = (hash) => parseOptions(String(hash).replace(/^#\??/, ''));

/** Query string options overlaid with hash options (hash wins). Same parsing for both, any key order. */
export function readParams(search, hash) {
  const params = parseOptions(String(search).replace(/^\?/, ''));
  for (const [k, v] of parseHash(hash)) params.set(k, v);
  return params;
}

const OPTION_TRUE = new Set(['', '1', 'true', 'yes', 'on']);
const OPTION_FALSE = new Set(['0', 'false', 'no', 'off']);

/**
 * Boolean URL option: true for a present key with no value ("#embed", "?embed", "embed=") or "1/true/yes/on"
 * (case-insensitive), false for "0/false/no/off", undefined when the key is absent or the value is unrecognised.
 */
export function optionFlag(params, key) {
  if (!params.has(key)) return undefined;
  const value = params.get(key).trim().toLowerCase();
  if (OPTION_TRUE.has(value)) return true;
  if (OPTION_FALSE.has(value)) return false;
  return undefined;
}

/**
 * Presentation mode from the URL options: the read-only viewer (embed or view on) unless edit on keeps the
 * editor. The inline script in index.html repeats this rule to hide the editing UI before any module runs;
 * tests/io/serialize.test.js keeps the two in agreement.
 */
export const isViewer = (params) =>
  (optionFlag(params, 'embed') === true || optionFlag(params, 'view') === true) && optionFlag(params, 'edit') !== true;

/**
 * The framing text of parsed URL options: `cam`, else a legacy `view=cx,cy,r` (a value with commas; the `view=1`
 * viewer flag never qualifies). Returns null when there is none; the text is not validated, decodeView() does that.
 */
export function framingText(params) {
  if (params.has(CAM_KEY)) return params.get(CAM_KEY);
  const legacy = params.get(LEGACY_KEY);
  return legacy !== null && legacy.includes(',') ? legacy : null;
}
