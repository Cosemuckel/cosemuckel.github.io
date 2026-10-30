// Shared JSDoc typedefs, for editor checking only (jsconfig.json); this module has no runtime content.
// Modules refer to them as import('../types.js').Name.

/** @typedef {{ x: number, y: number }} Point */

/**
 * A rigid body in its reference configuration. `pts` are the nodes; `segs` the index pairs of the drawn segments
 * (absent: the open chain through the points).
 * @typedef {{ id: number, kind: 'rod', pts: Point[], segs?: [number, number][] }} Body
 */

/**
 * A support located by the world point `p`; the bodies it acts on are derived from geometry.
 * `angleDeg`/`locked` apply to rollers, `clamped` to pins.
 * @typedef {{ id: number, type: 'pin' | 'roller' | 'joint', p: Point, angleDeg: number, locked: boolean, clamped: boolean }} Support
 */

/** @typedef {{ bodies: Body[], supports: Support[], nextId: number }} Scene */

/**
 * One generalised coordinate a mode can be driven by (see sim/drivers.js).
 * @typedef {{ id: string, kind: string, label: string, body: number, other?: number,
 *   value: (q: number[]) => number, gradient: (q?: number[]) => number[] }} Driver
 */

/**
 * Reachable values of one mode (see the header of sim/trajectory.js).
 * @typedef {{ kind: 'closed' | 'limited' | 'half' | 'unbounded', tMin: number, tMax: number, period?: number,
 *   status: 'closed' | 'blocked' | 'free' | 'budget', value: number, driverKind: string, [extra: string]: any }} ModeRange
 */

/**
 * Finite motion of a mechanism from a base pose (see sim/trajectory.js).
 * @typedef {{ count: number, modes: number[][], drivers: Driver[], candidates: { id: string, kind: string, label: string }[][],
 *   warnings: string[], poseAt: (t: number[], opts?: object) => { q: number[], params: number[], blocked: boolean },
 *   modeRange: (i: number) => ModeRange | null, [extra: string]: any }} Trajectory
 */

/** @typedef {{ cx: number, cy: number, scale: number }} ViewState */

/** @typedef {{ w: number, h: number, dpr?: number }} CanvasSize */

/**
 * What a shared link stores (io/serialize.js): the scene plus display flags, selected poles and driver choices.
 * @typedef {{ scene: Scene, flags: Record<string, boolean>, scale: number, mode: number, picks: string[],
 *   locked: boolean[], drivers?: string[] }} Setup
 */

/** @typedef {{ n: number, constraints: any[], [extra: string]: any }} System */

export {};
