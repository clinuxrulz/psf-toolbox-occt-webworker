#ifndef _UTIL_HPP_
#define _UTIL_HPP_

#include <gp_Vec.hxx>
#include <gp_Quaternion.hxx>

extern gp_Quaternion quaternion_from_uv(const gp_Vec& u, const gp_Vec& v);
extern gp_Quaternion quaternion_from_vw(const gp_Vec& v, const gp_Vec& w);
extern gp_Quaternion quaternion_from_wu(const gp_Vec& w, const gp_Vec& u);

#endif // _UTIL_HPP_
