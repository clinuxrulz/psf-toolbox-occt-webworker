#include "util.hpp"

gp_Quaternion quaternion_from_uvw(const gp_Vec& u, const gp_Vec& v, const gp_Vec& w);

gp_Quaternion quaternion_from_uv(const gp_Vec& u, const gp_Vec& v) {
    gp_Vec w = u.Crossed(v);
    w.Normalize();
    return quaternion_from_uvw(u, v, w);
}

gp_Quaternion quaternion_from_vw(const gp_Vec& v, const gp_Vec& w) {
    gp_Vec u = v.Crossed(w);
    u.Normalize();
    return quaternion_from_uvw(u, v, w);
}

gp_Quaternion quaternion_from_wu(const gp_Vec& w, const gp_Vec& u) {
    gp_Vec v = w.Crossed(u);
    v.Normalize();
    return quaternion_from_uvw(u, v, w);
}

gp_Quaternion quaternion_from_uvw(const gp_Vec& u, const gp_Vec& v, const gp_Vec& w) {
    double m00 = u.X();
    double m10 = u.Y();
    double m20 = u.Z();
    double m01 = v.X();
    double m11 = v.Y();
    double m21 = v.Z();
    double m02 = w.X();
    double m12 = w.Y();
    double m22 = w.Z();

    double tr = m00 + m11 + m22;
    double qw, qx, qy, qz;

    if (tr > 0) {
        double S = sqrt(tr + 1.0) * 2;
        qw = 0.25 * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if (m00 > m11 && m00 > m22) {
        double S = sqrt(1.0 + m00 - m11 - m22) * 2;
        qw = (m21 - m12) / S;
        qx = 0.25 * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        double S = sqrt(1.0 + m11 - m00 - m22) * 2;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25 * S;
        qz = (m12 + m21) / S;
    } else {
        double S = sqrt(1.0 + m22 - m00 - m11) * 2;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25 * S;
    }

    return gp_Quaternion(qx, qy, qz, qw);
}
