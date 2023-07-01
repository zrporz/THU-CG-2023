#ifndef TRIANGLE_H
#define TRIANGLE_H
#define MinLen 1e-6
#include "object3d.hpp"
#include <cmath>
#include <iostream>
#include <vecmath.h>
using namespace std;

// TODO: implement this class and add more fields as necessary
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a;
        vertices[1] = b;
        vertices[2] = c;
        Vector3f edge1 = a - c;
        Vector3f edge2 = b - a;
        normal = Vector3f::cross(edge1, edge2).normalized();
        d = Vector3f::dot(normal, a);
		e1sq = Vector3f::dot(edge1, edge1);
		e1e2 = Vector3f::dot(edge1, edge2);
		e2sq = Vector3f::dot(edge2, edge2);
		inverDeno = e1sq * e2sq - e1e2 * e1e2;
		is_norm = false;
		is_texture = false;
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		Vector3f o = ray.getOrigin();
        Vector3f dir = ray.getDirection();

		float dir_len = dir.length();
        dir.normalize();
        float c = Vector3f::dot(dir, normal);
        if (!c)
            return false;
        if (d - Vector3f::dot(o, normal) > -1e-3 && d - Vector3f::dot(o, normal) < 1e-3)
            return false;
        float t = (d - Vector3f::dot(normal, o)) / Vector3f::dot(normal, dir);
        t = t / dir_len;

        if(t > 0 && t > tmin && t < hit.getT()) {
			Vector3f edge1 = vertices[0] - vertices[2];
        	Vector3f edge2 = vertices[1] - vertices[0];

    		Vector3f ans = ray.pointAtParameter(t) - vertices[0];
			float e1ans = Vector3f::dot(edge1, ans);
			float e2ans = Vector3f::dot(edge2, ans);
			if(inverDeno == 0)
				return false;
			float u = (-e2sq * e1ans + e1e2 * e2ans) / inverDeno;

			if(u < 0 || u > 1)
				return false;

			float v = (e1sq * e2ans - e1e2 * e1ans) / inverDeno;

			if(v < 0 || v > 1)
				return false;
			if(u + v <= 1) {
				Vector3f next_origin = o + ray.getDirection() * t;
				float uu = 0, vv = 0;
				get_uv(next_origin, uu, vv);
				Vector3f new_normal = get_norm(next_origin);
            	hit.set(t, this->material, new_normal, material->get_color(uu, vv), next_origin, is_texture);
            	return true;
			}
        }

        return false;

		 
	}
	
	void get_uv(const Vector3f& p, float& u, float& v) {	//计算给定点 p 的纹理坐标
        if (!is_texture)
			return;
        Vector3f va = (vertices[0] - p), vb = (vertices[1] - p), vc = (vertices[2] - p);
        float ra = Vector3f::cross(vb, vc).length();
    	float rb = Vector3f::cross(vc, va).length();
    	float rc = Vector3f::cross(va, vb).length();
        Vector2f uv = (ra * at + rb * bt + rc * ct) / (ra + rb + rc); //加权平均
        u = uv.x();
        v = uv.y();
    }

	Vector3f get_norm(const Vector3f& p) {	//获取给定点 p 处的法线向
		if(!is_norm)
			return this->normal;
		Vector3f va = (vertices[0] - p), vb = (vertices[1] - p), vc = (vertices[2] - p);
		float ra = Vector3f::cross(vb, vc).length();
    	float rb = Vector3f::cross(vc, va).length();
    	float rc = Vector3f::cross(va, vb).length();
		return (ra * an + rb * bn + rc * cn).normalized();
	}


	Vector3f normal;
	Vector3f vertices[3];
	float minx, miny, minz, maxx, maxy, maxz;
protected:
	float d;
	float e1sq;
	float e1e2;
	float e2sq;
	float inverDeno;	// 用于计算相交参数的倒数
	bool is_texture;
	bool is_norm;
	Vector2f at, bt, ct;	// 纹理坐标的权重
    Vector3f an, bn, cn;
};

#endif // TRIANGLE_H
