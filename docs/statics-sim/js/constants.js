// Policy values: budgets, limits and timings that shape behaviour and are tuned in one place. Local numerical
// tolerances stay next to the maths that needs them. The values are part of the app's behaviour (and, for the
// motion budgets, of the golden test): change them deliberately.

// ---------------------------------------------------------------- motion (sim/)

/** Parameter distance of one integration step on a mechanism of diagonal <= 10 (checkpoint spacing along a walk). */
export const STEP = 0.1;
/** Longest walk (distance of the fastest point) that poseAt allows at least; longer when the search budget is longer. */
export const MAX_WALK = 200;
/** The search explores SEARCH_FACTOR x the bounding-box diagonal of the mechanism per direction. */
export const SEARCH_FACTOR = 8;
/** Upper bound of the integration steps spent on classifying one curve (both directions together). */
export const MAX_TOTAL_STEPS = 4000;
/** Fixed travel (in driver units) offered on a side of a mode that has no limit (free roller, translation). */
export const FREE_TRAVEL = 12;
/** Detached curves a trajectory keeps (the oldest is dropped first) while the driver selection changes. */
export const CACHED_CURVES = 8;

// ---------------------------------------------------------------- view and interaction (io/, app/)

/** Zoom limits in px per world unit; the wheel in app/ui.js and decodeView() both clamp to them. */
export const VIEW_SCALE = { min: 4, max: 300 };
/** Wheel zoom: the scale is multiplied by exp(-deltaY * WHEEL_ZOOM_RATE). */
export const WHEEL_ZOOM_RATE = 0.0015;
/** Animation frames for one pass across the range of a mode (driver units per frame = range / PLAY_FRAMES). */
export const PLAY_FRAMES = 360;

// ---------------------------------------------------------------- timings in ms (app/)

/** Longest slice of a range / driver-kind classification before the main thread is handed back. */
export const RANGE_SLICE_MS = 20;
/** Wait after the last mode change before the ranges of the other modes are recomputed for the new pose. */
export const STALE_RANGES_DEBOUNCE_MS = 150;
/** Wait after the last edit before the trajectory is rebuilt (edits arrive once per mouse move). */
export const TRAJECTORY_DEBOUNCE_MS = 150;
/** Retry delay of the driver-kind classification while slider ranges are still being computed. */
export const KIND_RETRY_MS = 50;
/** Wait after the last change before the setup / camera is written to the address bar. */
export const HASH_DEBOUNCE_MS = 400;
/** How long a message that needs reading (motion warnings and errors) stays in the status line. */
export const NOTICE_LONG_MS = 8000;
