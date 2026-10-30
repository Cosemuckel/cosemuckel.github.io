// World -> screen mapping of a frame (y points up in the world, down on the canvas).

/** Function mapping a world point to canvas px for the camera `view` ({cx, cy, scale}) on a w x h canvas. */
export const worldToScreen = (view, w, h) => (p) => ({ x: (p.x - view.cx) * view.scale + w / 2, y: h / 2 - (p.y - view.cy) * view.scale });

/** World direction -> screen direction. */
export const screenDir = (d) => ({ x: d.x, y: -d.y });
