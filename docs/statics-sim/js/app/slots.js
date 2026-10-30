// The mode slots of the current mechanism as the UI shows them: the driver of each "Mode N" (stored selection if it is
// valid here, otherwise the canonical default of sim/drivers.js) and the motion each slot stands for. No trajectory is
// needed, so the Options "Mode" selector and the displaced-shape display name and mean the same thing as the simulation
// panel's modes, whatever the pose or the numeric null-space basis.

import { defaultSelection, validateDriverSelection, slotMotions } from '../sim/drivers.js';

// Resolved drivers per system. The default selection is a function of the mechanism, so it is computed once per
// system (S.sys is rebuilt on every structural edit) and mobility, not once per drag frame.
const resolved = new WeakMap();

/**
 * { labels, motions } for the current pose of `state` (labels: driver label per slot; motions: null-space velocity per
 * slot; null when the slot drivers are singular at this pose, callers then fall back to the numeric null-space basis), or
 * null when the mechanism has no motion or no driver selection exists.
 */
export function slotsOf(state) {
  const { sys, q, analysis } = state;
  const k = analysis.nullity;
  if (!k) return null;
  const stored = state.sim.selection;
  let entry = resolved.get(sys);
  if (!entry || entry.k !== k || entry.stored !== stored) {
    const checked = stored ? validateDriverSelection(sys, q, stored, analysis.modes) : null;
    entry = { k, stored, drivers: checked && checked.ok ? checked.drivers : defaultSelection(sys, q, analysis.modes) };
    resolved.set(sys, entry);
  }
  if (!entry.drivers) return null;
  const motions = slotMotions(q, analysis.modes, entry.drivers);
  return { labels: entry.drivers.map((d) => d.label), motions };
}
