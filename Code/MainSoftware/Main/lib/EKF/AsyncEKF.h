/**
 * AsyncEKF.h - Generic asynchronous Extended Kalman Filter (header-only, fixed size, no heap).
 *
 * "Asynchronous" means there is no fixed filter rate: the state carries its own timestamp, and
 * every sensor is fused whenever its data arrives, at its own rate:
 *
 *   ekf.predictTo(model, tUs);              // propagate the state to the measurement time
 *   ekf.update(innovation, H, R, gate);     // fuse one sensor (any measurement size M)
 *
 * Measurements that arrive slightly late (timestamp before the state time, e.g. a barometer read
 * that finished after the next IMU sample was processed) are handled by syncTo(): if they are
 * no older than maxLagUs they are fused at the current state time, otherwise they are rejected.
 *
 * The process model is a user class with three methods (N = state size):
 *
 *   struct MyModel {
 *       void propagate(ekf::Vec<N>& x, float dt);                           // x <- f(x, u, dt)
 *       void jacobian(const ekf::Vec<N>& x, float dt, ekf::Mat<N,N>& F);    // F = df/dx
 *       void processNoise(const ekf::Vec<N>& x, float dt, ekf::Mat<N,N>& Q);
 *   };
 *
 * The model holds the current control input (e.g. the latest accelerometer sample), which is
 * applied as a zero-order hold until the next input arrives.
 *
 * Updates use the Joseph form (P stays symmetric positive-definite), an optional chi-square
 * innovation gate (outlier rejection), and are rolled back if they would produce NaN/Inf.
 */
#pragma once

#include <stdint.h>
#include "EKFMatrix.h"

namespace ekf {

/**
 * Normalized-innovation-squared (NIS) gates: reject a measurement whose NIS exceeds the value.
 * Chi-square quantiles for M = 1, 2, 3 degrees of freedom.
 */
constexpr float GATE_OFF = 0.0f;
constexpr float GATE_1DOF_99   = 6.63f,  GATE_1DOF_999 = 10.83f;   // ~2.6 / 3.3 sigma
constexpr float GATE_2DOF_99   = 9.21f,  GATE_2DOF_999 = 13.82f;
constexpr float GATE_3DOF_99   = 11.34f, GATE_3DOF_999 = 16.27f;

enum class UpdateStatus : uint8_t {
    Accepted,
    Gated,       // innovation too large for its uncertainty (outlier), state unchanged
    Stale,       // measurement older than maxLagUs, state unchanged
    Numerical,   // S not positive definite, or the result was not finite; state unchanged
    Skipped,     // deliberately not fused (e.g. used as a reference instead); state unchanged
};

struct UpdateResult {
    UpdateStatus status;
    float nis;     // normalized innovation squared y^T S^-1 y (expected value = M)
    bool accepted() const { return status == UpdateStatus::Accepted; }
};

template <int N>
class AsyncEKF {
public:
    using Vector = Vec<N>;
    using Matrix = Mat<N, N>;

    AsyncEKF() { reset(Vector::zeros(), Matrix::identity(), 0); }

    /** Sets the state, its covariance and the state's timestamp (microseconds). */
    void reset(const Vector& x0, const Matrix& P0, uint64_t tUs) {
        x_ = x0;
        P_ = P0;
        symmetrize(P_);
        tUs_ = tUs;
        predictions_ = accepted_ = gated_ = stale_ = numerical_ = 0;
    }

    // ---------------- State access ----------------
    const Vector& x() const { return x_; }
    const Matrix& P() const { return P_; }
    float x(int i) const { return x_[i]; }
    float stddev(int i) const { return sqrtf(P_(i, i) > 0.0f ? P_(i, i) : 0.0f); }
    uint64_t timeUs() const { return tUs_; }
    /** Direct write access, e.g. to clamp a state. Keep P consistent yourself. */
    Vector& mutableX() { return x_; }
    Matrix& mutableP() { return P_; }

    // ---------------- Settings ----------------
    /** Long prediction intervals are split into steps no longer than this (s). Default 10 ms. */
    void setMaxPredictStep(float seconds) { maxStep_ = seconds > 0.0f ? seconds : 0.01f; }
    /** Variance floor applied to the diagonal after every step (keeps the filter from locking up). */
    void setMinVariance(float v) { minVar_ = v; }
    /**
     * Covariance update form. Joseph (default) is the most robust and costs ~2 N^3 per update.
     * The short form P - K (P H^T)^T costs N^2 M, which matters for large N (e.g. a 15-state INS
     * fusing a baro at 1 kHz). P is symmetrized and floored either way.
     */
    void setJosephForm(bool on) { joseph_ = on; }

    // ---------------- Prediction ----------------
    /** Propagates the state by dt seconds (no timestamp change). */
    template <class Model>
    void predict(Model& model, float dt) {
        if (!(dt > 0.0f)) return;
        Matrix F, Q;
        model.jacobian(x_, dt, F);       // linearize at the state before propagation
        model.processNoise(x_, dt, Q);
        model.propagate(x_, dt);
        P_ = F * P_ * F.T() + Q;
        symmetrize(P_);
        floorVariance();
        predictions_++;
    }

    /**
     * Propagates the state forward to tUs (in steps <= maxPredictStep). Does nothing and returns
     * false if tUs is earlier than the current state time.
     */
    template <class Model>
    bool predictTo(Model& model, uint64_t tUs) {
        if (tUs < tUs_) return false;
        float remaining = (float)(tUs - tUs_) * 1e-6f;
        while (remaining > 0.0f) {
            float dt = remaining > maxStep_ ? maxStep_ : remaining;
            predict(model, dt);
            remaining -= dt;
        }
        tUs_ = tUs;
        return true;
    }

    /**
     * Brings the filter to a measurement taken at tUs: predicts forward if it is newer than the
     * state, accepts it as-is if it is late by no more than maxLagUs, otherwise returns false
     * (and counts it as stale).
     */
    template <class Model>
    bool syncTo(Model& model, uint64_t tUs, uint32_t maxLagUs) {
        if (tUs >= tUs_) return predictTo(model, tUs);
        if (tUs_ - tUs <= maxLagUs) return true;
        stale_++;
        return false;
    }

    // ---------------- Measurement update ----------------
    /**
     * Fuses one measurement of size M.
     * @param y    innovation z - h(x) (computed by the caller, so angles can be wrapped)
     * @param H    measurement Jacobian dh/dx (M x N)
     * @param R    measurement noise covariance (M x M)
     * @param gate NIS threshold (GATE_* constants), or GATE_OFF to accept everything
     */
    template <int M>
    UpdateResult update(const Vec<M>& y, const Mat<M, N>& H, const Mat<M, M>& R, float gate = GATE_OFF) {
        const Mat<N, M> PHt = P_ * H.T();
        Mat<M, M> S = H * PHt + R;
        symmetrize(S);
        Mat<M, M> Sinv;
        if (!choleskyInverse(S, Sinv)) {
            numerical_++;
            return {UpdateStatus::Numerical, NAN};
        }
        const float nis = (y.T() * Sinv * y)(0, 0);
        if (gate > 0.0f && !(nis <= gate)) {
            gated_++;
            return {UpdateStatus::Gated, nis};
        }

        const Mat<N, M> K = PHt * Sinv;
        const Vector xNew = x_ + K * y;
        Matrix PNew;
        if (joseph_) {
            // Joseph form: P = (I - KH) P (I - KH)^T + K R K^T  (stays symmetric positive definite)
            const Matrix IKH = Matrix::identity() - K * H;
            PNew = IKH * P_ * IKH.T() + K * R * K.T();
        } else {
            // Short form: P = P - K H P = P - K (P H^T)^T
            PNew = P_ - K * PHt.T();
        }
        symmetrize(PNew);

        if (!xNew.allFinite() || !PNew.allFinite()) {
            numerical_++;
            return {UpdateStatus::Numerical, nis};
        }
        x_ = xNew;
        P_ = PNew;
        floorVariance();
        accepted_++;
        return {UpdateStatus::Accepted, nis};
    }

    /**
     * Like update(), but only the states with mask[i] = true may change (a "consider" update).
     * Use it when a sensor should not correct some states even though they are correlated with
     * it, e.g. so that a barometer corrects altitude but not attitude, where an unmodeled vertical
     * error would otherwise be "explained" by rotating the attitude. The gain is then suboptimal,
     * so P is updated with the general (Joseph) form, expanded to cost N^2 M:
     *   P = P - K U^T - U K^T + K S K^T,  U = P H^T
     */
    template <int M>
    UpdateResult updateMasked(const Vec<M>& y, const Mat<M, N>& H, const Mat<M, M>& R, const bool (&mask)[N],
                              float gate = GATE_OFF) {
        const Mat<N, M> U = P_ * H.T();
        Mat<M, M> S = H * U + R;
        symmetrize(S);
        Mat<M, M> Sinv;
        if (!choleskyInverse(S, Sinv)) {
            numerical_++;
            return {UpdateStatus::Numerical, NAN};
        }
        const float nis = (y.T() * Sinv * y)(0, 0);
        if (gate > 0.0f && !(nis <= gate)) {
            gated_++;
            return {UpdateStatus::Gated, nis};
        }

        Mat<N, M> K = U * Sinv;
        for (int i = 0; i < N; i++)
            if (!mask[i])
                for (int j = 0; j < M; j++) K(i, j) = 0.0f;
        const Vector xNew = x_ + K * y;
        const Mat<N, N> KUt = K * U.T();
        Matrix PNew = P_ - KUt - KUt.T() + K * S * K.T();
        symmetrize(PNew);

        if (!xNew.allFinite() || !PNew.allFinite()) {
            numerical_++;
            return {UpdateStatus::Numerical, nis};
        }
        x_ = xNew;
        P_ = PNew;
        floorVariance();
        accepted_++;
        return {UpdateStatus::Accepted, nis};
    }

    /** Convenience for scalar measurements. */
    UpdateResult update1(float y, const Mat<1, N>& H, float variance, float gate = GATE_OFF) {
        Vec<1> yv;
        yv[0] = y;
        Mat<1, 1> R;
        R(0, 0) = variance;
        return update<1>(yv, H, R, gate);
    }

    // ---------------- Statistics ----------------
    uint32_t predictions() const { return predictions_; }
    uint32_t acceptedUpdates() const { return accepted_; }
    uint32_t gatedUpdates() const { return gated_; }
    uint32_t staleUpdates() const { return stale_; }
    uint32_t numericalFailures() const { return numerical_; }

private:
    void floorVariance() {
        for (int i = 0; i < N; i++)
            if (P_(i, i) < minVar_) P_(i, i) = minVar_;
    }

    Vector x_;
    Matrix P_;
    uint64_t tUs_ = 0;
    float maxStep_ = 0.01f;
    float minVar_ = 1e-9f;
    bool joseph_ = true;
    uint32_t predictions_ = 0, accepted_ = 0, gated_ = 0, stale_ = 0, numerical_ = 0;
};

} // namespace ekf
