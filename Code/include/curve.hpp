#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <utility>
#include <vecmath.h>
#include <vector>

#include <algorithm>
const float INF= 0x3f3f3f3f;
// TODO (PA2): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};

class Curve : public Object3D {
public:
    std::vector<Vector3f> controls;
    int n, k; // n: number of control points, k: number of degree
    std::vector<double> t, tpad; // t: knot vector
    std::vector<double> s, ds;
    float ymin, ymax, radius;
    int lsk;
    double range[2];
    explicit Curve(std::vector<Vector3f> points)
        : controls(std::move(points))
    {
        ymin = INF;
        ymax = -INF;
        radius = 0;
        for (auto pt : controls) {
            ymin = min(pt.y(), ymin);
            ymax = max(pt.y(), ymax);
            radius = max(radius, fabs(pt.x()));
            radius = max(radius, fabs(pt.z()));
        }
    }

    bool intersect(const Ray& r, Hit& h, float tmin) override
    {
        return false;
    }

    std::vector<Vector3f>& getControls()
    {
        return controls;
    }
    int get_bpos(double mu)
    {
        // 找到不大于mu的最大元素下标
        return upper_bound(t.begin(), t.end(), mu) - t.begin() - 1;
    }
    std::pair<double,double> get_valid_range(){
        return std::make_pair(t[k],t[n]);
    }
    void evaluate(double mu){
        int bpos = get_bpos(mu);
            s = std::vector<double> (k + 2, 0), ds = std::vector<double> (k + 1, 1);
            s[k] = 1;
            for (int p = 1; p < k + 1; p++) {
                for (int ii = k - p; ii < k + 1; ii++) {
                    int i = ii + bpos - k;
                    double w1, dw1, w2, dw2;
                    if (tpad[i + p] == tpad[i]) {
                        w1 = mu;
                        dw1 = 1;
                    } else {
                        w1 = (mu - tpad[i]) / (tpad[i + p] - tpad[i]);
                        dw1 = 1.0 / (tpad[i + p] - tpad[i]);
                    }
                    if (tpad[i + p + 1] == tpad[i + 1]) {
                        w2 = 1 - mu;
                        dw2 = -1;
                    } else {
                        w2 = (tpad[i + p + 1] - mu) / (tpad[i + p + 1] - tpad[i + 1]);
                        dw2 = -1 / (tpad[i + p + 1] - tpad[i + 1]);
                    }
                    if (p == k)
                        ds[ii] = (dw1 * s[ii] + dw2 * s[ii + 1]) * p;
                    s[ii] = w1 * s[ii] + w2 * s[ii + 1];
                }
            }
            s.pop_back();
            lsk = bpos-k; int rsk = n-bpos - 1;
            if (lsk < 0) {
                s.erase(s.begin(),s.begin()-lsk);
                ds.erase(ds.begin(),ds.begin()-lsk);
                lsk = 0;
            }
            if (rsk < 0) {
                s.erase(s.end()+rsk,s.end());
                ds.erase(ds.end()+rsk,ds.end());
            }
    }
    virtual void discretize(int resolution, std::vector<CurvePoint>& data) = 0;

};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f>& points)
        : Curve(points)
    {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
        n = controls.size();
        k = n - 1;
        range[0] = 0;
        range[1] = 1;
        t.resize(2 * n);
        for (int i = 0; i < n; i++) {
            t[i] = 0;
            t[i + n] = 1;
        }
        int tSize = t.size();
        tpad.resize(tSize + k);
        for (int i = 0; i < tSize + k; i++) 
            tpad[i] = i<tSize ? t[i] : t.back();
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override
    {
        // TODO (PA2): fill in data vector
        resolution *= n/k;
        data.resize(resolution);
        for (int i = 0; i < resolution; i++) {
            data[i].T = Vector3f::ZERO;
            data[i].V = Vector3f::ZERO;
            std::pair<double,double> range = get_valid_range();
            t.resize(2*n);
            double mu = ((double)i / resolution) * (range.second-range.first) + range.first;
            evaluate(mu);
            for (int j = 0; j < s.size(); ++j) {
                data[i].V += controls[lsk + j] * s[j];
                data[i].T += controls[lsk + j] * ds[j];
            }
        }
    }

};

class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f>& points)
        : Curve(points)
    {
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
        n = controls.size();
        k = 3;
        t.resize(n + k + 1);
        for (int i = 0; i < n + k + 1; ++i)
            t[i] = (double)i / (n + k);
        int tSize = t.size();
        tpad.resize(tSize + k);
        for (int i = 0; i < tSize + k; i++) 
            tpad[i] = i<tSize ? t[i] : t.back();
        range[0] = t[k];
        range[1] = t[n];
    }

    void discretize(int resolution, std::vector<CurvePoint>& data) override
    {
        // TODO (PA2): fill in data vector
        resolution *= n / k;
        data.resize(resolution);
        for (int i = 0; i < resolution; i++) {
            data[i].T = Vector3f::ZERO;
            data[i].V = Vector3f::ZERO;
            std::pair<double,double> range = get_valid_range();
            double mu = ((double)i / resolution) * (range.second-range.first) + range.first;
            evaluate(mu);
            for (int j = 0; j < s.size(); ++j) {
                data[i].V += controls[lsk + j] * s[j];
                data[i].T += controls[lsk + j] * ds[j];
            }
        }
    }
};

#endif // CURVE_HPP
