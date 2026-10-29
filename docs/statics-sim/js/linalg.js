// Small dense linear-algebra helpers (plain arrays, no DOM).

/**
 * Solve the square system A x = b by Gauss-Jordan elimination with partial pivoting.
 * Returns null when A is numerically singular.
 */
export function solve(A, b) {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let best = c;
    for (let i = c + 1; i < n; i++) if (Math.abs(M[i][c]) > Math.abs(M[best][c])) best = i;
    if (Math.abs(M[best][c]) < 1e-14) return null;
    [M[c], M[best]] = [M[best], M[c]];
    const pv = M[c][c];
    for (let j = c; j <= n; j++) M[c][j] /= pv;
    for (let i = 0; i < n; i++) {
      if (i === c) continue;
      const f = M[i][c];
      if (f === 0) continue;
      for (let j = c; j <= n; j++) M[i][j] -= f * M[c][j];
    }
  }
  return M.map((row) => row[n]);
}

const dotv = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

/**
 * Orthonormal basis of the null space {x : A x = 0} of an m x nCols matrix.
 * Uses reduced row echelon form (columns whose pivot is below `tol` are free),
 * then Gram-Schmidt on the resulting basis vectors.
 */
export function nullspace(rows, nCols, tol = 1e-7) {
  const A = rows.map((r) => r.slice());
  const m = A.length;
  const pivotCols = [];
  let r = 0;
  for (let c = 0; c < nCols && r < m; c++) {
    let best = r;
    for (let i = r + 1; i < m; i++) if (Math.abs(A[i][c]) > Math.abs(A[best][c])) best = i;
    if (Math.abs(A[best][c]) <= tol) continue;
    [A[r], A[best]] = [A[best], A[r]];
    const pv = A[r][c];
    for (let j = 0; j < nCols; j++) A[r][j] /= pv;
    for (let i = 0; i < m; i++) {
      if (i === r) continue;
      const f = A[i][c];
      if (f === 0) continue;
      for (let j = 0; j < nCols; j++) A[i][j] -= f * A[r][j];
    }
    pivotCols.push(c);
    r++;
  }
  const isPivot = new Set(pivotCols);
  const basis = [];
  for (let c = 0; c < nCols; c++) {
    if (isPivot.has(c)) continue;
    const v = new Array(nCols).fill(0);
    v[c] = 1;
    pivotCols.forEach((pc, k) => {
      v[pc] = -A[k][c];
    });
    basis.push(v);
  }
  // Gram-Schmidt
  const out = [];
  for (const v of basis) {
    const w = v.slice();
    for (const u of out) {
      const k = dotv(w, u);
      for (let i = 0; i < nCols; i++) w[i] -= k * u[i];
    }
    const l = Math.sqrt(dotv(w, w));
    if (l > 1e-12) out.push(w.map((x) => x / l));
  }
  return out;
}
