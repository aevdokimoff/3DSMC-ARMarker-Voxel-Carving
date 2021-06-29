#ifndef IMPLICIT_SURFACE_H
#define IMPLICIT_SURFACE_H

#include "../simple_mesh.h"

class ImplicitSurface
{
public:
    virtual double Eval(const Vec3d& x) = 0;
};


class Sphere : public ImplicitSurface
{
public:
    Sphere(const Vec3d& center, double radius) : m_center(center), m_radius(radius) {}

    ~Sphere() = default;

    double Eval(const Vec3d& _x) override;

private:
    Vec3d m_center;
    double m_radius;
};


class Torus : public ImplicitSurface
{
public:
    Torus(const Vec3d& center, double radius, double a) : m_center(center), m_radius(radius), m_a(a) {}

    ~Torus() = default;

    double Eval(const Vec3d& _x) override;

private:
    Vec3d m_center;
    double m_radius;
    double m_a;
};

#endif
