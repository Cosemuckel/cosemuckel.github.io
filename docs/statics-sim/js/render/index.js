// Canvas rendering: the public API of js/render/.
//   render(ctx, frame)   draws one frame (see scene.js for the frame object)
//   traceLongLine        cuts a long dashed line to the canvas (exported for tests/render/render.test.js)

export { drawScene as render } from './scene.js';
export { traceLongLine } from './poles.js';
