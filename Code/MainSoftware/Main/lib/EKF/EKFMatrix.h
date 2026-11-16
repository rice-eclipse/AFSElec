/**
 * EKFMatrix.h - Minimal fixed-size float matrices for the EKF (no heap, no dependencies).
 *
 * Sizes are template parameters, so dimension mistakes are compile errors. Only what a Kalman
 * filter needs is here: +, -, *, transpose, identity, symmetrize and a Cholesky inverse.
 */
#pragma once

#include <math.h>
#include <string.h>

namespace ekf {

template <int R, int C>
struct Mat {
    static_assert(R > 0 && C > 0, "matrix dimensions must be positive");
    float a[R][C];

    static Mat zeros() {
        Mat m;
        memset(m.a, 0, sizeof m.a);
        return m;
    }
    static Mat identity() {
        static_assert(R == C, "identity() needs a square matrix");
        Mat m = zeros();
        for (int i = 0; i < R; i++) m.a[i][i] = 1.0f;
        return m;
    }
    /** Diagonal matrix from a list of values (square only). */
    static Mat diagonal(const float (&d)[R]) {
        static_assert(R == C, "diagonal() needs a square matrix");
        Mat m = zeros();
        for (int i = 0; i < R; i++) m.a[i][i] = d[i];
        return m;
    }

    float& operator()(int r, int c) { return a[r][c]; }
    float operator()(int r, int c) const { return a[r][c]; }
    // Vector-style element access (row-major flat index), for column vectors
    float& operator[](int i) { return (&a[0][0])[i]; }
    float operator[](int i) const { return (&a[0][0])[i]; }

    Mat<C, R> T() const {
        Mat<C, R> t;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++) t.a[c][r] = a[r][c];
        return t;
    }

    Mat operator+(const Mat& o) const {
        Mat m;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++) m.a[r][c] = a[r][c] + o.a[r][c];
        return m;
    }
    Mat operator-(const Mat& o) const {
        Mat m;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++) m.a[r][c] = a[r][c] - o.a[r][c];
        return m;
    }
    Mat operator*(float s) const {
        Mat m;
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++) m.a[r][c] = a[r][c] * s;
        return m;
    }
    Mat& operator+=(const Mat& o) { return *this = *this + o; }
    Mat& operator-=(const Mat& o) { return *this = *this - o; }

    bool allFinite() const {
        for (int r = 0; r < R; r++)
            for (int c = 0; c < C; c++)
                if (!isfinite(a[r][c])) return false;
        return true;
    }
};

template <int N>
using Vec = Mat<N, 1>;

template <int R, int K, int C>
inline Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) {
    Mat<R, C> m;
    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            float s = 0.0f;
            for (int k = 0; k < K; k++) s += x.a[r][k] * y.a[k][c];
            m.a[r][c] = s;
        }
    }
    return m;
}

/** Makes a square matrix exactly symmetric: (A + A^T) / 2. */
template <int N>
inline void symmetrize(Mat<N, N>& A) {
    for (int r = 0; r < N; r++) {
        for (int c = r + 1; c < N; c++) {
            float v = 0.5f * (A.a[r][c] + A.a[c][r]);
            A.a[r][c] = A.a[c][r] = v;
        }
    }
}

/**
 * Inverse of a symmetric positive-definite matrix via Cholesky (A = L L^T, A^-1 = L^-T L^-1).
 * Returns false if A is not positive definite (or not finite).
 */
template <int N>
inline bool choleskyInverse(const Mat<N, N>& A, Mat<N, N>& inv) {
    // L: lower-triangular Cholesky factor
    Mat<N, N> L = Mat<N, N>::zeros();
    for (int j = 0; j < N; j++) {
        float d = A.a[j][j];
        for (int k = 0; k < j; k++) d -= L.a[j][k] * L.a[j][k];
        if (!(d > 0.0f) || !isfinite(d)) return false;
        float ljj = sqrtf(d);
        L.a[j][j] = ljj;
        for (int i = j + 1; i < N; i++) {
            float s = A.a[i][j];
            for (int k = 0; k < j; k++) s -= L.a[i][k] * L.a[j][k];
            L.a[i][j] = s / ljj;
        }
    }
    // Li = L^-1 (lower triangular, forward substitution)
    Mat<N, N> Li = Mat<N, N>::zeros();
    for (int i = 0; i < N; i++) {
        Li.a[i][i] = 1.0f / L.a[i][i];
        for (int j = 0; j < i; j++) {
            float s = 0.0f;
            for (int k = j; k < i; k++) s -= L.a[i][k] * Li.a[k][j];
            Li.a[i][j] = s / L.a[i][i];
        }
    }
    inv = Li.T() * Li;
    return true;
}

} // namespace ekf
