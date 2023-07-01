#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <cmath>
#include <vecmath.h>

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D {
public:
    Sphere()
        : radius(0)
        , center(0, 0, 0)
    {
        // unit ball at the center
    }

    Sphere(const Vector3f& center, float radius, Material* material, Vector3f v = Vector3f::ZERO)
        : Object3D(material)
        , radius(radius)
        , center(center)
        , velocity(v)
    {
        move = (v  != Vector3f::ZERO);
    }

    ~Sphere() override = default;

    bool intersect(const Ray& r, Hit& h, float tmin) override
    {
        Vector3f o = move ? (r.getOrigin() - RND2*velocity) : r.getOrigin();    //运动模糊，相当于光源向运动的反方向偏移
        Vector3f dir = r.getDirection().normalized(), o2c = center - o;
        float oh = Vector3f::dot(o2c, dir);
        float ch = o2c.squaredLength() - oh * oh;
        if (ch > radius*radius) {
            // {fprintf(stderr,"%f,%f\n",ch,radius);
            return false;
        }

        float ph = sqrt(radius * radius - ch);
        float t = oh - ph;

        // 如果此次相交算出的 t 比上次的大，则说明本次相交并不是离相机最近的，应该舍弃，否则应该相应地更新 t,如果 t < tmin，本次相交也应该舍弃
        if (t < tmin || t > h.getT())
            return false;

        // 计算交点处的法向量
        Vector3f P = r.pointAtParameter(t);
        Vector3f normal = (P - center).normalized(); // 计算相交处法向量

        // 更新 Hit 对象
        float u = 0.5 + atan2(normal.x(), normal.z()) / (2 * M_PI);
        float v = 0.5 - asin(normal.y()) / M_PI;

        Vector2f grad(0, 0); // 存储凹凸贴图的梯度信息
        float f = material->bump.get_disturb(u, v, grad); // 计算给定贴图坐标(u, v)处的扰动值

        if (f > 1e-4 || f < -1e-4) {    // f较小时不做法向扰动
            float phi, theta; // 根据贴图坐标(u, v)计算球面上的纹理坐标。
            phi = u * 2 * M_PI;
            theta = M_PI - v * M_PI;
            // 通过扰动梯度信息改变法向量的方向
            Vector3f pu(-normal.z(), 0, normal.x()), pv(normal.y() * cos(phi), -radius * sin(theta), normal.y() * sin(phi));

            normal = Vector3f::cross(pu + normal * grad[0] / (2 * M_PI), pv + normal * grad[1] / M_PI).normalized();
        }

        h.set(t, material, normal, material->get_color(u, v), P, material->texture.is_texture);
        return true;
    }

protected:
    Vector3f center;
    Vector3f velocity;
    float radius;
    bool move;
};


#endif
