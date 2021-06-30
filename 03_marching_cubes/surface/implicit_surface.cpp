#include "implicit_surface.h"

double Sphere::Eval(const Vec3d& _x)
{
    return pow(_x[0] - m_center[0], 2) +
           pow(_x[1] - m_center[1], 2) +
           pow(_x[2] - m_center[2], 2) -
           pow(m_radius, 2);
}


double Torus::Eval(const Vec3d& _x)
{
    double xSquared = pow(_x[0] - m_center[0], 2);
    double ySquared = pow(_x[1] - m_center[1], 2);
    double zSquared = pow(_x[2] - m_center[2], 2);
    return pow(xSquared + ySquared + zSquared + pow(m_radius, 2) - pow(m_a, 2), 2) -
           4 * pow(m_radius, 2) * (xSquared + ySquared);
}