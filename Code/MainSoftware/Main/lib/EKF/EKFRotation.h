/**
 * EKFRotation.h - 3-vectors, quaternions and rotation matrices for the EKF (header-only, float).
 *
 * Conventions (used by NavEKF):
 *   - Quaternion q = [w, x, y, z], Hamilton product, rotates body -> world:  v_world = q * v_body * q^-1
 *   - R = toMatrix(q) is the same rotation as a matrix:  v_world = R v_body
 *   - Small rotations are rotation vectors (axis * angle, rad); exp() / fromRotationVector()
 *     turn them into quaternions.
 *   - Euler angles are aerospace ZYX (yaw, then pitch, then roll), in radians.
 */
#pragma once

#include <math.h>
#include "EKFMatrix.h"

namespace ekf {

using Vec3 = Vec<3>;
using Mat3 = Mat<3, 3>;

inline Vec3 vec3(float x, float y, float z) {
    Vec3 v;
    v[0] = x;
    v[1] = y;
    v[2] = z;
    return v;
}
inline float dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return vec3(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
inline float norm(const Vec3& a) { return sqrtf(dot(a, a)); }
/** a / |a|, or zeros if |a| is 0. */
inline Vec3 normalized(const Vec3& a) {
    const float n = norm(a);
    return n > 0.0f ? a * (1.0f / n) : Vec3::zeros();
}
/** Skew-symmetric (cross-product) matrix: skew(a) * b = a x b. */
inline Mat3 skew(const Vec3& a) {
    Mat3 m = Mat3::zeros();
    m(0, 1) = -a[2]; m(0, 2) = a[1];
    m(1, 0) = a[2];  m(1, 2) = -a[0];
    m(2, 0) = -a[1]; m(2, 1) = a[0];
    return m;
}

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    static Quat identity() { return Quat(); }
    static Quat of(float w, float x, float y, float z) {
        Quat q;
        q.w = w; q.x = x; q.y = y; q.z = z;
        return q;
    }

    Quat operator*(const Quat& b) const {   // Hamilton product
        return of(w * b.w - x * b.x - y * b.y - z * b.z,
                  w * b.x + x * b.w + y * b.z - z * b.y,
                  w * b.y - x * b.z + y * b.w + z * b.x,
                  w * b.z + x * b.y - y * b.x + z * b.w);
    }
    Quat conj() const { return of(w, -x, -y, -z); }

    Quat normalized() const {
        const float n = sqrtf(w * w + x * x + y * y + z * z);
        if (!(n > 0.0f)) return identity();
        const float s = (w < 0.0f ? -1.0f : 1.0f) / n;   // keep w >= 0 (same rotation)
        return of(w * s, x * s, y * s, z * s);
    }

    /** Rotation by the rotation vector r (axis * angle, rad). */
    static Quat fromRotationVector(const Vec3& r) {
        const float a2 = dot(r, r);
        float c, s;   // cos(angle/2), sin(angle/2)/angle
        if (a2 < 1e-8f) {   // Taylor series, avoids 0/0
            c = 1.0f - a2 / 8.0f;
            s = 0.5f - a2 / 48.0f;
        } else {
            const float a = sqrtf(a2);
            c = cosf(0.5f * a);
            s = sinf(0.5f * a) / a;
        }
        return of(c, r[0] * s, r[1] * s, r[2] * s);
    }

    /** Rotation vector (axis * angle) of this quaternion, angle in [0, pi]. */
    Vec3 toRotationVector() const {
        const Quat q = normalized();
        const float sv = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
        if (sv < 1e-6f) return vec3(2.0f * q.x, 2.0f * q.y, 2.0f * q.z);
        const float k = 2.0f * atan2f(sv, q.w) / sv;
        return vec3(q.x * k, q.y * k, q.z * k);
    }

    /** Rotation matrix R (v_world = R v_body). */
    Mat3 toMatrix() const {
        Mat3 R;
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;
        R(0, 0) = 1 - 2 * (yy + zz); R(0, 1) = 2 * (xy - wz);     R(0, 2) = 2 * (xz + wy);
        R(1, 0) = 2 * (xy + wz);     R(1, 1) = 1 - 2 * (xx + zz); R(1, 2) = 2 * (yz - wx);
        R(2, 0) = 2 * (xz - wy);     R(2, 1) = 2 * (yz + wx);     R(2, 2) = 1 - 2 * (xx + yy);
        return R;
    }

    /** Quaternion from a proper rotation matrix (Shepperd's method). */
    static Quat fromMatrix(const Mat3& R) {
        const float tr = R(0, 0) + R(1, 1) + R(2, 2);
        Quat q;
        if (tr > 0.0f) {
            const float s = 2.0f * sqrtf(1.0f + tr);
            q = of(0.25f * s, (R(2, 1) - R(1, 2)) / s, (R(0, 2) - R(2, 0)) / s, (R(1, 0) - R(0, 1)) / s);
        } else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
            const float s = 2.0f * sqrtf(1.0f + R(0, 0) - R(1, 1) - R(2, 2));
            q = of((R(2, 1) - R(1, 2)) / s, 0.25f * s, (R(0, 1) + R(1, 0)) / s, (R(0, 2) + R(2, 0)) / s);
        } else if (R(1, 1) > R(2, 2)) {
            const float s = 2.0f * sqrtf(1.0f + R(1, 1) - R(0, 0) - R(2, 2));
            q = of((R(0, 2) - R(2, 0)) / s, (R(0, 1) + R(1, 0)) / s, 0.25f * s, (R(1, 2) + R(2, 1)) / s);
        } else {
            const float s = 2.0f * sqrtf(1.0f + R(2, 2) - R(0, 0) - R(1, 1));
            q = of((R(1, 0) - R(0, 1)) / s, (R(0, 2) + R(2, 0)) / s, (R(1, 2) + R(2, 1)) / s, 0.25f * s);
        }
        return q.normalized();
    }

    /** From aerospace ZYX Euler angles [rad]. */
    static Quat fromEuler(float roll, float pitch, float yaw) {
        const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
        const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
        const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
        return of(cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                  cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy);
    }

    /** Rotates a vector body -> world. */
    Vec3 rotate(const Vec3& v) const { return toMatrix() * v; }

    /** Angle of the rotation between two attitudes [rad], in [0, pi]. */
    static float angleBetween(const Quat& a, const Quat& b) { return norm((a.conj() * b).toRotationVector()); }
};

} // namespace ekf
