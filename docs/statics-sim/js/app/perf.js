// Development-only frame timing, switched on with "?perf=1" (or "#perf=1"). Off by default: ui.js creates nothing and
// wraps nothing unless the switch is set, so there is no cost when it is off.
//
// timed(label, fn) wraps fn and records its wall time (performance.now()). Every REPORT_EVERY samples of a label a
// summary row (count, mean, max in ms) is printed with console.table; all samples stay readable in window.__perf.

const REPORT_EVERY = 60;

export function createPerfLog() {
  const samples = new Map(); // label -> number[]
  const unreported = new Map(); // label -> number[] since the last table
  const summary = (xs) => ({
    count: xs.length,
    'mean ms': +(xs.reduce((a, b) => a + b, 0) / xs.length).toFixed(2),
    'max ms': +Math.max(...xs).toFixed(2),
  });
  const log = {
    samples,
    add(label, ms) {
      if (!samples.has(label)) {
        samples.set(label, []);
        unreported.set(label, []);
      }
      samples.get(label).push(ms);
      const fresh = unreported.get(label);
      fresh.push(ms);
      if (fresh.length >= REPORT_EVERY || label === 'rebuildTrajectory') {
        console.table({ [label]: summary(fresh) });
        fresh.length = 0;
      }
    },
    /** fn with its duration recorded under `label` (also when it throws). */
    timed(label, fn) {
      return (...args) => {
        const start = performance.now();
        try {
          return fn(...args);
        } finally {
          log.add(label, performance.now() - start);
        }
      };
    },
  };
  window.__perf = log;
  return log;
}
