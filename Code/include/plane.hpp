#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <cmath>
#include <vecmath.h>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane()
    {
    }

    Plane(const Vector3f& normal, float d, Material* m, const Vector3f& v = Vector3f::ZERO)
        : Object3D(m)
    {
        this->d = d / normal.length();
        this->normal = normal.normalized();
        if (normal.x() < 1e-3 && normal.z() < 1e-3) {
            this->main_tangent = Vector3f::RIGHT;
        } else {
            this->main_tangent = Vector3f::cross(Vector3f::UP, normal);
            this->main_tangent.normalize();
        }

        this->bi_normal = Vector3f::cross(main_tangent, normal);
        this->bi_normal.normalize();
    }

    ~Plane() override = default;

    void get_uv(const Vector3f& p, float& u, float& v)
    {
        v = Vector3f::dot(p - d * normal, bi_normal) / 100;
        u = Vector3f::dot(p - d * normal, main_tangent) / 100;
    }

    bool intersect(const Ray& r, Hit& h, float tmin) override
    {
        Vector3f o = r.getOrigin();
        Vector3f dir = r.getDirection();
        float dir_len = dir.length();
        dir.normalize();
        float c = Vector3f::dot(dir, normal);
        if (!c)
            return false;
        if (d - Vector3f::dot(o, normal) > -1e-3 && d - Vector3f::dot(o, normal) < 1e-3)
            return false;
        float t = (d - Vector3f::dot(normal, o)) / Vector3f::dot(normal, dir);
        t = t / dir_len;
        if (t < tmin || t > h.getT()) {
            return false;
        }
        Vector3f next_o = o + r.getDirection() * t;
        float v = next_o.y();
        float u = Vector3f::dot(next_o - r.getDirection() * normal, main_tangent);
        Vector2f grad = Vector2f::ZERO;
        float f = material->bump.get_disturb(u, v, grad);

        Vector3f new_normal = normal;

        if (!(f < 1e-4 && f > -1e-4)) {
            new_normal += main_tangent * grad[0];
            new_normal += bi_normal * grad[1];
            new_normal.normalize();
        }

        float uu = 0, vv = 0;
        get_uv(next_o, uu, vv);

        h.set(t, this->material, new_normal, material->get_color(uu, vv), next_o, material->texture.is_texture);
        return true;
    }

protected:
    Vector3f normal;
    Vector3f main_tangent;
    Vector3f bi_normal;
    float d;
};

#endif // PLANE_H
