// @ts-check
// Small dense linear-algebra helpers (plain arrays, no DOM).
//
// The matrices of this app are sparse (a constraint row touches the 3 coordinates of one or two bodies), so the
// elimination loops below only visit non-zero entries of the pivot row. Skipping `x - f * 0` leaves every result
// bit-for-bit what the plain dense loops give.

/** Indices j in [from, to) with row[j] !== 0. */
function nonZeroIndices(row, from, to) {
  const out = [];
  for (let j = from; j < to; j++) if (row[j] !== 0) out.push(j);
  return out;
}

/**
 * Solve the square system A x = b by Gauss-Jordan elimination with partial pivoting.
 * Returns null when A is numerically singular.
 */
export function solve(A, b) {
  const n = b.length;
  const M = new Array(n);
  for (let i = 0; i < n; i++) {
    const row = new Float64Array(n + 1);
    const src = A[i];
    for (let j = 0; j < n; j++) row[j] = src[j];
    row[n] = b[i];
    M[i] = row;
  }
  for (let c = 0; c < n; c++) {
    let best = c;
    for (let i = c + 1; i < n; i++) if (Math.abs(M[i][c]) > Math.abs(M[best][c])) best = i;
    if (Math.abs(M[best][c]) < 1e-14) return null;
    [M[c], M[best]] = [M[best], M[c]];
    const pivotRow = M[c];
    const pv = pivotRow[c];
    for (let j = c; j <= n; j++) pivotRow[j] /= pv;
    const cols = nonZeroIndices(pivotRow, c, n + 1);
    for (let i = 0; i < n; i++) {
      if (i === c) continue;
      const row = M[i];
      const f = row[c];
      if (f === 0) continue;
      for (let k = 0; k < cols.length; k++) {
        const j = cols[k];
        row[j] -= f * pivotRow[j];
      }
    }
  }
  const x = new Array(n);
  for (let i = 0; i < n; i++) x[i] = M[i][n];
  return x;
}

/** Inverse of the square matrix A (Gauss-Jordan, partial pivoting); null when A is numerically singular. */
export function inverse(A) {
  const n = A.length;
  const M = A.map((row, i) => {
    const out = new Float64Array(2 * n);
    for (let j = 0; j < n; j++) out[j] = row[j];
    out[n + i] = 1;
    return out;
  });
  for (let c = 0; c < n; c++) {
    let best = c;
    for (let i = c + 1; i < n; i++) if (Math.abs(M[i][c]) > Math.abs(M[best][c])) best = i;
    if (Math.abs(M[best][c]) < 1e-14) return null;
    [M[c], M[best]] = [M[best], M[c]];
    const pivotRow = M[c];
    const pv = pivotRow[c];
    for (let j = 0; j < 2 * n; j++) pivotRow[j] /= pv;
    for (let i = 0; i < n; i++) {
      if (i === c) continue;
      const row = M[i];
      const f = row[c];
      if (f === 0) continue;
      for (let j = 0; j < 2 * n; j++) row[j] -= f * pivotRow[j];
    }
  }
  return M.map((row) => Array.from(row.subarray(n)));
}

function dotv(a, b) {
  let s = 0;
  for (let i = 0; i < a.length; i++) s += a[i] * b[i];
  return s;
}

/**
 * Orthonormal basis of the null space {x : A x = 0} of an m x nCols matrix.
 * Uses reduced row echelon form (columns whose pivot is below `tol` are free),
 * then Gram-Schmidt on the resulting basis vectors.
 */
export function nullspace(rows, nCols, tol = 1e-7) {
  const m = rows.length;
  const A = new Array(m);
  for (let i = 0; i < m; i++) A[i] = Float64Array.from(rows[i]);
  const pivotCols = [];
  let r = 0;
  for (let c = 0; c < nCols && r < m; c++) {
    let best = r;
    for (let i = r + 1; i < m; i++) if (Math.abs(A[i][c]) > Math.abs(A[best][c])) best = i;
    if (Math.abs(A[best][c]) <= tol) continue;
    [A[r], A[best]] = [A[best], A[r]];
    const pivotRow = A[r];
    const pv = pivotRow[c];
    for (let j = 0; j < nCols; j++) pivotRow[j] /= pv;
    const cols = nonZeroIndices(pivotRow, 0, nCols);
    for (let i = 0; i < m; i++) {
      if (i === r) continue;
      const row = A[i];
      const f = row[c];
      if (f === 0) continue;
      for (let k = 0; k < cols.length; k++) {
        const j = cols[k];
        row[j] -= f * pivotRow[j];
      }
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
    if (l > 1e-12) {
      for (let i = 0; i < nCols; i++) w[i] /= l;
      out.push(w);
    }
  }
  return out;
}
