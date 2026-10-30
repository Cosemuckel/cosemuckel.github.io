// @ts-check
// Resumable computations (pure, no DOM). A long computation is written as a generator that yields between small units
// of work; runSync() runs it to the end (the synchronous API), stepFor() runs it for a time slice so that a caller can
// hand the main thread back between slices.

/** Runs a generator to completion and returns its result. */
export function runSync(gen) {
  for (;;) {
    const step = gen.next();
    if (step.done) return step.value;
  }
}

/**
 * Advances a generator for about `budgetMs` milliseconds (always at least one unit of work).
 * Returns { done, value }; `value` is the generator's result once `done`.
 */
export function stepFor(gen, budgetMs, now = () => performance.now()) {
  const end = now() + budgetMs;
  for (;;) {
    const step = gen.next();
    if (step.done || now() >= end) return step;
  }
}
