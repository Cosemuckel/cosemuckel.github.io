// Text on the canvas.

/** Text with a white halo so it stays readable over lines and bodies. */
export function labelText(ctx, text, x, y, { color, font, align = 'left' }) {
  ctx.font = font;
  ctx.textAlign = align;
  ctx.lineJoin = 'round';
  ctx.lineWidth = 3.5;
  ctx.strokeStyle = 'rgba(255,255,255,0.92)';
  ctx.strokeText(text, x, y);
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}
