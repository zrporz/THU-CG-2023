#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include "utils.hpp"
#include <cmath>
#include <float.h>
#include <vecmath.h>

class Camera {
public:
    Camera(const Vector3f& center, const Vector3f& direction, const Vector3f& up, int imgW, int imgH)
    {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f& point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f& center, const Vector3f& direction,
        const Vector3f& up, int imgW, int imgH, float angle)
        : Camera(center, direction, up, imgW, imgH)
    {
        cx = imgW >> 1;
        cy = imgH >> 1;
        // 根据角度计算放缩值
        fx = cx / tan(angle / 2);
        fy = cy / tan(angle / 2);
        // angle is in radian.
    }

    Ray generateRay(const Vector2f& point) override
    {
        Vector3f drc((point.x() - cx) / fx, (cy - point.y()) / fy, 1);
        return Ray(center, Matrix3f(horizontal, -up, direction) * drc.normalized());
    }

protected:
    float fx, fy, cx, cy;
};

class DofCamera : public Camera {
    float fxy;
    float distance; // 成像平面到其的距离
    float radius; // 相机光圈半径
public:
    DofCamera(const Vector3f& center, const Vector3f& direction,
        const Vector3f& up, int imgW, int imgH, float angle, float distance, float radius)
        : Camera(center, direction, up, imgW, imgH)
    {
        // angle is in radian.
        cx = imgW >> 1;
        cy = imgH >> 1;
        // 根据角度计算放缩值
        fx = cx / tan(angle / 2);
        fy = cy / tan(angle / 2);
        fxy = imgH / (2 * tan(angle / 2) * (distance + 1));
        this->distance = distance; // 在真实空间的距离
        this->radius = radius; // 在真实空间的半径
    }

    Ray generateRay(const Vector2f& point) override
    {

        float csx = distance * (point.x() - cx) / fx;
        float csy = distance * (point.y() - cy) / fy;
        float theta = 2 * M_PI * RND2;
        float r = RND2 * radius;
        Vector3f point2 = r * sin(theta) * up + r * cos(theta) * horizontal;
        Vector3f dir(csx - point2.x(), -csy - point2.y(), distance);
        return Ray(center + horizontal * point2.x() - up * point2.y(), Matrix3f(horizontal, -up, direction) * dir.normalized());
    }

protected:
    float fx;
    float fy;
    float cx;
    float cy;
};
#endif // CAMERA_H
