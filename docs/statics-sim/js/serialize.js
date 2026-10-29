// Scene setup <-> URL-safe string (pure: no DOM, no storage).
//
// A "setup" is the shareable state of the app:
//   { scene: { bodies, supports, nextId },      model.js shapes
//     flags: { beams: true, ... },              display toggles named in VIEW_FLAGS
//     scale, mode,                              displacement scale and selected mode
//     picks: [supportId, ...],                  picked pole beams / poles
//     locked: [bool, ...] }                     per-mode lock state of the simulation
//
// Wire format: one prefix character + base64url text.
//   "b" + base64url(bytes)                        plain
//   "z" + base64url(deflate-raw(bytes))           when CompressionStream exists and it is shorter
// The bytes are a compact binary layout (see packSetup) that stores only the drawn state: ids are
// implicit (bodies are numbered in order, then supports), coordinates are small integers in a
// per-scene unit, and view/simulation state is written only where it differs from the defaults.
// Everything decoded is untrusted input: unpackSetup validates counts, ranges and finiteness and
// throws SetupError. Version 1 (JSON) links are rejected with a clear message.

export const FORMAT_VERSION = 2;

/** Hard bounds applied when decoding (and exceeded only by hostile or corrupt input). */
export const LIMITS = {
  bodies: 200,
  pointsPerBody: 500,
  segsPerBody: 1000,
  supports: 400,
  picks: 400,
  modes: 64,
  coord: 1e6,
  encodedChars: 120000,
  rawBytes: 600000,
};

/** Display flags stored in the setup, in a fixed order (bit i of the mask is VIEW_FLAGS[i]); append only. */
export const VIEW_FLAGS = ['grid', 'beams', 'inters', 'kennedy', 'poleLabels', 'bodyLabels', 'displaced', 'original', 'vectors'];
/** Flag bits above VIEW_FLAGS that older links may carry (the removed "bodyPoles" display); read and ignored. */
const RETIRED_FLAG_BITS = 1;
/** Flag values of a fresh editor; a setup only stores the flags when they differ from this. */
export const DEFAULT_FLAGS = {
  grid: true, beams: true, inters: true, kennedy: true, poleLabels: true, bodyLabels: true,
  displaced: false, original: true, vectors: false,
};
export const DEFAULT_SCALE = 0.5;
export const SCALE_RANGE = 3;

export class SetupError extends Error {
  constructor(message) {
    super(message);
    this.name = 'SetupError';
  }
}
const fail = (message) => {
  throw new SetupError(message);
};

const flagMask = (flags) => VIEW_FLAGS.reduce((mask, name, i) => (flags[name] ? mask + 2 ** i : mask), 0);
const DEFAULT_MASK = flagMask(DEFAULT_FLAGS);

// Coordinates are written as integers in units of 1/UNITS[k]; the first unit that represents every
// coordinate of the scene exactly is used, else 1/1000 (coordinates are then rounded to 1e-3).
const UNITS = [1, 2, 4, 5, 10, 20, 100, 1000];
const EXACT = 1e-7;
const SUPPORT_TYPES = ['pin', 'roller', 'joint'];
// Header "present" byte: bits 0-2 = unit index, the rest say which optional sections follow.
const HAS_FLAGS = 8;
const HAS_SCALE = 16;
const HAS_MODE = 32;
const HAS_PICKS = 64;
const HAS_LOCKS = 128;
// Support header byte: bits 0-1 type, 2 sits on a body node, 3 locked, 4 clamped, 5 pole beam hidden.
const SUP_ON_NODE = 4;
const SUP_LOCKED = 8;
const SUP_CLAMPED = 16;
const SUP_NO_BEAM = 32;
const MAX_ANGLE_DEG = 360000; // a roller angle is a free number; this only keeps it sane

// --------------------------------------------------------------------- byte coding

const zigzag = (n) => (n < 0 ? -2 * n - 1 : 2 * n);
const unzigzag = (u) => (u % 2 ? -(u + 1) / 2 : u / 2);

/** Unsigned LEB128 varints and zigzag signed integers (no bit operators: values may exceed 32 bits). */
class ByteWriter {
  constructor() {
    this.bytes = [];
  }
  byte(v) {
    this.bytes.push(v);
  }
  uint(n) {
    let v = n;
    while (v >= 128) {
      this.bytes.push((v % 128) + 128);
      v = Math.floor(v / 128);
    }
    this.bytes.push(v);
  }
  int(n) {
    this.uint(zigzag(n));
  }
}

class ByteReader {
  constructor(bytes) {
    this.bytes = bytes;
    this.at = 0;
  }
  byte() {
    if (this.at >= this.bytes.length) fail('link data is truncated');
    return this.bytes[this.at++];
  }
  uint(max, what) {
    let v = 0;
    for (let shift = 0; shift < 7; shift++) {
      const b = this.byte();
      v += (b % 128) * 128 ** shift;
      if (b < 128) {
        if (v > max) fail(`${what} is out of range (${v}, limit ${max})`);
        return v;
      }
    }
    return fail(`${what} is malformed`);
  }
  int(maxMagnitude, what) {
    return unzigzag(this.uint(2 * maxMagnitude + 1, what));
  }
  get done() {
    return this.at === this.bytes.length;
  }
}

// ------------------------------------------------------------------ pack / unpack

const sameNode = (a, b) => Math.abs(a.x - b.x) < 1e-9 && Math.abs(a.y - b.y) < 1e-9;
const isPlainChain = (segs, pointCount) => segs.length === pointCount - 1 && segs.every(([a, b], i) => a === i && b === i + 1);

/** Smallest unit index under which every value is an integer, else the finest (values then get rounded). */
function chooseUnit(values) {
  const k = UNITS.findIndex((u) => values.every((v) => Math.abs(v * u - Math.round(v * u)) < EXACT));
  return k < 0 ? UNITS.length - 1 : k;
}

/**
 * Setup -> bytes. Layout (u = varint, i = zigzag varint):
 *   u version, byte present|unit, [u flagMask] [i scale*100] [u mode] [u n, n * u pickedSupportIndex] [u n, ceil(n/8) lock-bit bytes]
 *   u bodyCount, per body: u (pointCount * 2 + explicitSegs), pointCount * (i dx, i dy), [u segCount, segCount * (u a, u b)]
 *   u supportCount, per support: byte header, (u nodeIndex | i dx, i dy), [roller: u angleCode]
 * Points and free support positions are deltas from the previously written position, in units of 1/UNITS[unit].
 * Segments are omitted when they are the plain chain through the points. A support on a body node
 * stores the node's index (over all bodies' points in order) instead of coordinates. A roller's angle
 * is written as degrees when integral, else as thousandths of a degree (angleCode = zigzag * 2 + fractional);
 * other support types do not store an angle (it has no meaning for them).
 */
export function packSetup(setup) {
  const { scene } = setup;
  const nodes = scene.bodies.flatMap((b) => b.pts);
  const nodeIndexOf = (p) => nodes.findIndex((q) => sameNode(p, q));
  const supportNode = scene.supports.map((s) => nodeIndexOf(s.p));
  const values = [
    ...nodes.flatMap((p) => [p.x, p.y]),
    ...scene.supports.filter((_, i) => supportNode[i] < 0).flatMap((s) => [s.p.x, s.p.y]),
  ];
  const unitIndex = chooseUnit(values);
  const unit = UNITS[unitIndex];

  const mask = flagMask(setup.flags);
  const picked = scene.supports.map((s, i) => (setup.picks.includes(s.id) ? i : -1)).filter((i) => i >= 0);
  const scaleCode = Math.round(setup.scale * 100);
  let present = unitIndex;
  if (mask !== DEFAULT_MASK) present |= HAS_FLAGS;
  if (scaleCode !== DEFAULT_SCALE * 100) present |= HAS_SCALE;
  if (setup.mode) present |= HAS_MODE;
  if (picked.length) present |= HAS_PICKS;
  if (setup.locked.some(Boolean)) present |= HAS_LOCKS;

  const w = new ByteWriter();
  w.uint(FORMAT_VERSION);
  w.byte(present);
  if (present & HAS_FLAGS) w.uint(mask);
  if (present & HAS_SCALE) w.int(scaleCode);
  if (present & HAS_MODE) w.uint(setup.mode);
  if (present & HAS_PICKS) {
    w.uint(picked.length);
    picked.forEach((i) => w.uint(i));
  }
  if (present & HAS_LOCKS) {
    w.uint(setup.locked.length);
    for (let i = 0; i < setup.locked.length; i += 8) {
      w.byte(setup.locked.slice(i, i + 8).reduce((bits, on, k) => (on ? bits + 2 ** k : bits), 0));
    }
  }

  let px = 0;
  let py = 0;
  const putPoint = (p) => {
    const x = Math.round(p.x * unit);
    const y = Math.round(p.y * unit);
    w.int(x - px);
    w.int(y - py);
    px = x;
    py = y;
  };
  w.uint(scene.bodies.length);
  for (const b of scene.bodies) {
    const explicit = !!b.segs && !isPlainChain(b.segs, b.pts.length);
    w.uint(b.pts.length * 2 + (explicit ? 1 : 0));
    b.pts.forEach(putPoint);
    if (explicit) {
      w.uint(b.segs.length);
      for (const [a, c] of b.segs) {
        w.uint(a);
        w.uint(c);
      }
    }
  }
  w.uint(scene.supports.length);
  scene.supports.forEach((s, i) => {
    const onNode = supportNode[i] >= 0;
    w.byte(SUPPORT_TYPES.indexOf(s.type) | (onNode ? SUP_ON_NODE : 0) | (s.locked ? SUP_LOCKED : 0) | (s.clamped ? SUP_CLAMPED : 0) | (s.showBeam === false ? SUP_NO_BEAM : 0));
    if (onNode) w.uint(supportNode[i]);
    else putPoint(s.p);
    if (s.type === 'roller') {
      const milli = Math.round(s.angleDeg * 1000);
      const whole = milli % 1000 === 0;
      w.uint(zigzag(whole ? milli / 1000 : milli) * 2 + (whole ? 0 : 1));
    }
  });
  return Uint8Array.from(w.bytes);
}

/** Bytes -> validated setup. Throws SetupError for anything malformed. */
export function unpackSetup(bytes) {
  if (bytes[0] === 0x7b) fail('this link was made by an older version of the app (version 1) and can no longer be opened');
  const r = new ByteReader(bytes);
  const version = r.uint(2 ** 32, 'version');
  if (version !== FORMAT_VERSION) fail(`unsupported setup version ${version} (this app reads version ${FORMAT_VERSION})`);
  const present = r.byte();
  const unit = UNITS[present & 7];
  const maxInt = LIMITS.coord * unit;

  const mask = present & HAS_FLAGS ? r.uint(2 ** (VIEW_FLAGS.length + RETIRED_FLAG_BITS) - 1, 'display flags') : DEFAULT_MASK;
  const flags = {};
  VIEW_FLAGS.forEach((name, i) => {
    flags[name] = Math.floor(mask / 2 ** i) % 2 === 1;
  });
  const scale = present & HAS_SCALE ? r.int(SCALE_RANGE * 100, 'displacement scale') / 100 : DEFAULT_SCALE;
  const mode = present & HAS_MODE ? r.uint(LIMITS.modes - 1, 'mode') : 0;
  const pickIndexes = [];
  if (present & HAS_PICKS) {
    const n = r.uint(LIMITS.picks, 'pick count');
    for (let i = 0; i < n; i++) pickIndexes.push(r.uint(LIMITS.supports - 1, 'pick index'));
  }
  const locked = [];
  if (present & HAS_LOCKS) {
    const n = r.uint(LIMITS.modes, 'mode lock count');
    for (let i = 0; i < n; i += 8) {
      const bits = r.byte();
      for (let k = 0; k < 8 && i + k < n; k++) locked.push(Math.floor(bits / 2 ** k) % 2 === 1);
    }
  }

  let px = 0;
  let py = 0;
  const readPoint = (what) => {
    px += r.int(2 * maxInt, `${what} x`);
    py += r.int(2 * maxInt, `${what} y`);
    if (Math.abs(px) > maxInt || Math.abs(py) > maxInt) fail(`${what} is outside the allowed range (|x| <= ${LIMITS.coord})`);
    return { x: px / unit, y: py / unit };
  };

  const bodyCount = r.uint(LIMITS.bodies, 'body count');
  const bodies = [];
  const nodes = [];
  for (let i = 0; i < bodyCount; i++) {
    const what = `body ${i + 1}`;
    const head = r.uint(LIMITS.pointsPerBody * 2 + 1, `${what} point count`);
    const pointCount = Math.floor(head / 2);
    if (pointCount < 2) fail(`${what} has too few points`);
    const pts = Array.from({ length: pointCount }, () => readPoint(what));
    nodes.push(...pts);
    const body = { id: i + 1, kind: 'rod', pts };
    if (head % 2) {
      const n = r.uint(LIMITS.segsPerBody, `${what} segment count`);
      if (!n) fail(`${what} has no segments`);
      body.segs = [];
      for (let k = 0; k < n; k++) {
        const a = r.uint(pointCount - 1, `${what} segment index`);
        const b = r.uint(pointCount - 1, `${what} segment index`);
        if (a === b) fail(`${what} has a zero-length segment`);
        body.segs.push([a, b]);
      }
    } else if (pointCount > 2) {
      // The scene builders (model.addPolyline) store explicit segments for multi-segment bodies; match them.
      body.segs = pts.slice(1).map((_, k) => [k, k + 1]);
    }
    bodies.push(body);
  }

  const supportCount = r.uint(LIMITS.supports, 'support count');
  const supports = [];
  for (let i = 0; i < supportCount; i++) {
    const what = `support ${i + 1}`;
    const head = r.uint(63, `${what} header`);
    const type = SUPPORT_TYPES[head & 3];
    if (!type) fail(`${what} has an unknown type`);
    let p;
    if (head & SUP_ON_NODE) {
      if (!nodes.length) fail(`${what} refers to a body node but there are no bodies`);
      p = { ...nodes[r.uint(nodes.length - 1, `${what} node index`)] };
    } else {
      p = readPoint(what);
    }
    let angleDeg = 0;
    if (type === 'roller') {
      const code = r.uint(zigzag(MAX_ANGLE_DEG * 1000) * 2 + 1, `${what} angle`);
      const value = unzigzag(Math.floor(code / 2));
      angleDeg = code % 2 ? value / 1000 : value;
      if (Math.abs(angleDeg) > MAX_ANGLE_DEG) fail(`${what} angle is out of range`);
    }
    supports.push({
      id: bodyCount + i + 1,
      type,
      p,
      angleDeg,
      locked: !!(head & SUP_LOCKED),
      clamped: !!(head & SUP_CLAMPED),
      showBeam: !(head & SUP_NO_BEAM),
    });
  }
  if (!r.done) fail('link data has unexpected trailing bytes');

  const picks = [];
  for (const index of pickIndexes) {
    if (index >= supportCount) fail(`pick index ${index} refers to a missing support`);
    if (!picks.includes(supports[index].id)) picks.push(supports[index].id);
  }
  return {
    scene: { bodies, supports, nextId: bodyCount + supportCount + 1 },
    flags,
    scale,
    mode,
    picks,
    locked,
  };
}

// --------------------------------------------------------------------- base64url

export function bytesToBase64Url(bytes) {
  let bin = '';
  for (let i = 0; i < bytes.length; i += 0x2000) bin += String.fromCharCode(...bytes.subarray(i, i + 0x2000));
  return btoa(bin).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

export function base64UrlToBytes(text) {
  if (!/^[A-Za-z0-9_-]*$/.test(text) || text.length % 4 === 1) fail('link text is not valid base64url');
  const std = text.replace(/-/g, '+').replace(/_/g, '/');
  const bin = atob(std + '='.repeat((4 - (std.length % 4)) % 4));
  return Uint8Array.from(bin, (c) => c.charCodeAt(0));
}

// -------------------------------------------------------------------- compression

const canCompress = () => typeof CompressionStream === 'function' && typeof Blob === 'function';
const canDecompress = () => typeof DecompressionStream === 'function' && typeof Blob === 'function';

/** Runs bytes through a (de)compression stream, refusing to produce more than maxOut bytes. */
async function throughStream(bytes, stream, maxOut) {
  const reader = new Blob([bytes]).stream().pipeThrough(stream).getReader();
  const chunks = [];
  let total = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    total += value.length;
    if (total > maxOut) {
      await reader.cancel();
      fail(`link expands to more than ${maxOut} bytes`);
    }
    chunks.push(value);
  }
  const out = new Uint8Array(total);
  let at = 0;
  for (const c of chunks) {
    out.set(c, at);
    at += c.length;
  }
  return out;
}

// ------------------------------------------------------------------- encode / decode

/**
 * Setup -> URL-safe string ("b..." plain or "z..." deflated, whichever is shorter).
 * Pass { compress: false } to force the plain form.
 */
export async function encodeSetup(setup, { compress = true } = {}) {
  const raw = packSetup(setup);
  const plain = 'b' + bytesToBase64Url(raw);
  if (!compress || !canCompress()) return plain;
  const packed = 'z' + bytesToBase64Url(await throughStream(raw, new CompressionStream('deflate-raw'), LIMITS.rawBytes));
  return packed.length < plain.length ? packed : plain;
}

/** URL-safe string -> validated setup. Rejects (SetupError) malformed, oversized, out-of-range or version 1 input. */
export async function decodeSetup(text) {
  if (typeof text !== 'string' || !text) fail('the link contains no setup');
  if (text.length > LIMITS.encodedChars) fail(`link is too long (${text.length} characters, limit ${LIMITS.encodedChars})`);
  const kind = text[0];
  if (kind === 'j') fail('this link was made by an older version of the app (version 1) and can no longer be opened');
  if (kind !== 'b' && kind !== 'z') fail('unknown setup encoding');
  let bytes = base64UrlToBytes(text.slice(1));
  if (kind === 'z') {
    if (!canDecompress()) fail('this browser cannot read compressed links (no DecompressionStream)');
    try {
      bytes = await throughStream(bytes, new DecompressionStream('deflate-raw'), LIMITS.rawBytes);
    } catch (err) {
      if (err instanceof SetupError) throw err;
      throw new SetupError('compressed link data is corrupt');
    }
  }
  if (bytes.length > LIMITS.rawBytes) fail(`setup is too large (${bytes.length} bytes)`);
  return unpackSetup(bytes);
}

// ------------------------------------------------------------------- URL options

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
 * tests/serialize.test.js keeps the two in agreement.
 */
export const isViewer = (params) =>
  (optionFlag(params, 'embed') === true || optionFlag(params, 'view') === true) && optionFlag(params, 'edit') !== true;

/**
 * Hash text (without "#") for the given hash options with the encoded setup stored under `s`.
 * `example` and `displaced` are dropped: the setup carries the scene and the display flags.
 */
export function hashWithSetup(hashParams, encoded, { drop = [] } = {}) {
  const p = new URLSearchParams(hashParams);
  for (const k of ['example', 'displaced', ...drop]) p.delete(k);
  p.set('s', encoded);
  return p.toString();
}
