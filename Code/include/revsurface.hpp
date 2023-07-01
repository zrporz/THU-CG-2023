#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include <tuple>

#include "curve.hpp"
#include "object3d.hpp"
#include "triangle.hpp"
#include "constants.hpp"
#include <math.h>

class AABB {
public:
    AABB()
    {
        bounds[0] = Vector3f(INF);
        bounds[1] = Vector3f(-INF);
    }

    // 初始化AABB
    AABB(const Vector3f& min, const Vector3f& max)
    {
        bounds[0] = min;
        bounds[1] = max;
    }

    // 设置AABB
    void set(const Vector3f& lo, const Vector3f& hi)
    {
        bounds[0] = lo;
        bounds[1] = hi;
    }

    // 更新AABB的边界
    void updateBound(const Vector3f& vec)
    {
        for (int i = 0; i < 3; ++i) {
            bounds[0][i] = bounds[0][i] < vec[i] ? bounds[0][i] : vec[i];
            bounds[1][i] = bounds[1][i] < vec[i] ? vec[i] : bounds[1][i];
            bounds[0][i] = bounds[0][i] < vec[i] ? bounds[0][i] : vec[i];
            bounds[1][i] = bounds[1][i] < vec[i] ? vec[i] : bounds[1][i];
        }
    }

    // 检测AABB是否与指定的Ray相交
    bool intersect(const Ray& r, float& t_min)
    {
        Vector3f o(r.getOrigin()), invdir(1 / r.getDirection());
        vector<int> sgn = { invdir.x() < 0, invdir.y() < 0, invdir.z() < 0 };
        t_min = INF;
        float tmin, tmax, tymin, tymax, tzmin, tzmax;
        tmin = (bounds[sgn[0]].x() - o.x()) * invdir.x();
        tmax = (bounds[1 - sgn[0]].x() - o.x()) * invdir.x();
        tymin = (bounds[sgn[1]].y() - o.y()) * invdir.y();
        tymax = (bounds[1 - sgn[1]].y() - o.y()) * invdir.y();
        if ((tmin > tymax) || (tymin > tmax))
            return false;
        if (tymin > tmin)
            tmin = tymin;
        if (tymax < tmax)
            tmax = tymax;
        tzmin = (bounds[sgn[2]].z() - o.z()) * invdir.z();
        tzmax = (bounds[1 - sgn[2]].z() - o.z()) * invdir.z();
        if ((tmin > tzmax) || (tzmin > tmax))
            return false;
        if (tzmin > tmin)
            tmin = tzmin;
        if (tzmax < tmax)
            tmax = tzmax;
        t_min = tmin;
        return true;
    }
    // 边界
    Vector3f bounds[2];
};

class RevSurface : public Object3D {
    Curve* pCurve;
    AABB aabb;
    // Definition for drawable surface.
    typedef std::tuple<unsigned, unsigned, unsigned> Tup3u;
    std::vector<Triangle> triangles;

public:
    RevSurface(Curve* pCurve, Material* material)
        : pCurve(pCurve)
        , Object3D(material)
    {
        // Check flat.
        for (const auto& cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        aabb.set(Vector3f(-pCurve->radius, pCurve->ymin - 3, -pCurve->radius),
            Vector3f(pCurve->radius, pCurve->ymax + 3, pCurve->radius));
    }

    ~RevSurface() override { delete pCurve; }

    inline bool intersect(const Ray& r, Hit& h, float tmin) override
    {
        return newtonIntersect(r, h);
    }

    bool newtonIntersect(const Ray& r, Hit& h)
    {
        float t, theta, mu;
        // 检测射线与包围盒的相交性
        if (!aabb.intersect(r, t) || t > h.getT())
            return false; // 检测射线r是否与某个包围盒相交
        // 检测射线与包围盒的相交性
        getUV(r, t, theta, mu); // 计算射线与曲线的参数值theta和mu。
        Vector3f normal, point;
        // 利用牛顿迭代法求交点和法线
        if (!newton(r, t, theta, mu, normal, point)) {
            return false;
        }
        // 检查参数值的有效性
        if (!isnormal(mu) || !isnormal(theta) || !isnormal(t))
            return false;
        if (t < 0 || mu < pCurve->range[0] || mu > pCurve->range[1] || t > h.getT())
            return false;
        // 设置Hit对象的属性
        h.set(t, material, normal.normalized(),
            material->get_color(theta / (2 * M_PI), mu), point, material->texture.is_texture);
        return true;
    }

    bool newton(const Ray& r, float& t, float& theta, float& mu,
        Vector3f& normal, Vector3f& point)
    { // 牛顿迭代法 theta:位置参数 [0,2\pi] mu:位置比例 [0,1]
        // 求解交点和法线
        Vector3f dmu, dtheta;
        for (int i = 0; i < newton_depth; ++i) {
            // 规范化角度
            if (theta < 0.0)
                theta += 2 * M_PI;
            if (theta >= 2 * M_PI)
                theta = fmod(theta, 2 * M_PI);
            // 限制参数范围
            if (mu >= 1)
                mu = 1.0 - FLT_EPSILON;
            if (mu <= 0)
                mu = FLT_EPSILON;
            // 计算交点和切向量
            point = getPoint(theta, mu, dtheta, dmu);
            Vector3f f = r.getOrigin() + r.getDirection() * t - point;
            float dist2 = f.squaredLength();
            normal = Vector3f::cross(dmu, dtheta);
            if (dist2 < NEWTON_EPS)
                return true;
            // 牛顿迭代更新参数
            float D = Vector3f::dot(r.getDirection(), normal);
            t -= Vector3f::dot(dmu, Vector3f::cross(dtheta, f)) / D;
            mu -= Vector3f::dot(r.getDirection(), Vector3f::cross(dtheta, f)) / D;
            theta += Vector3f::dot(r.getDirection(), Vector3f::cross(dmu, f)) / D;
        }
        return false;
    }

    void getUV(const Ray& r, const float& t, float& theta, float& mu)
    {
        Vector3f pt(r.getOrigin() + r.getDirection() * t);
        theta = atan2(-pt.z(), pt.x()) + M_PI;
        mu = (pCurve->ymax - pt.y()) / (pCurve->ymax - pCurve->ymin);
    }

    Vector3f getPoint(const float& theta, const float& mu, Vector3f& dtheta,
        Vector3f& dmu)
    {
        //根据给定的参数值theta和mu，计算参数曲面上对应点的位置，并计算该点处的切向量关于theta和mu的偏导数。
        Vector3f pt;
        Quat4f rot;
        rot.setAxisAngle(theta, Vector3f::UP);
        Matrix3f rotMat = Matrix3f::rotation(rot);
        CurvePoint cp;
        pCurve->evaluate(mu);
        for (int j = 0; j < pCurve->s.size(); ++j) {
            cp.V += pCurve->controls[pCurve->lsk + j] * pCurve->s[j];
            cp.T += pCurve->controls[pCurve->lsk + j] * pCurve->ds[j];
        }
        pt = rotMat * cp.V;
        dmu = rotMat * cp.T;
        dtheta = Vector3f(-cp.V.x() * sin(theta), 0, -cp.V.x() * cos(theta));
        return pt;
    }

};

#endif // REVSURFACE_HPP
