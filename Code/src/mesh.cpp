#include "mesh.hpp"
#include "constants.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

using namespace std;

float now_center;

bool Pel_cen_x(Pel x);
bool Pel_cen_y(Pel x);
bool Pel_cen_z(Pel x);
bool Pel_min_x(Pel x, Pel y);
bool Pel_min_y(Pel x, Pel y);
bool Pel_min_z(Pel x, Pel y);
bool Pel_max_x(Pel x, Pel y);
bool Pel_max_y(Pel x, Pel y);
bool Pel_max_z(Pel x, Pel y);
BVH_TreeNode::BVH_TreeNode(BVH_TreeNode* a, vector<Pel>::iterator it, int _num)
{
    parent = a;
    lc = rc = nullptr;
    // rc = nullptr;
    first_pel = it;
    pel_num = _num;

    if (_num) {
        minxyz = Vector3f((*min_element(it, it + _num, Pel_min_x)).centroid.x(), (*min_element(it, it + _num, Pel_min_y)).centroid.y(), (*min_element(it, it + _num, Pel_min_z)).centroid.z());
        maxxyz = Vector3f((*max_element(it, it + _num, Pel_max_x)).centroid.x(), (*max_element(it, it + _num, Pel_max_y)).centroid.y(), (*max_element(it, it + _num, Pel_max_z)).centroid.z());
    }
}

bool Ray_hit_AABB(BVH_TreeNode* e, const Ray& r, float& the_t)
{
    // 判断光线是否与轴对齐边界框（AABB）相交并计算交点参数 the_t

    if (e->pel_num == 0)
        return false;

    Vector3f origin = r.getOrigin();
    Vector3f direction = r.getDirection().normalized();

    float t_xmin, t_xmax, t_ymin, t_ymax, t_zmin, t_zmax;
    t_xmin = t_ymin = t_zmin = -1e38;
    t_xmax = t_ymax = t_zmax = 1e38;

    // 计算光线与 AABB 边界的交点参数 t
    if (direction.x() > 1e-4) {
        t_xmin = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmax = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if (direction.x() < -1e-4) {
        t_xmax = (e->minxyz.x() - origin.x()) / direction.x();
        t_xmin = (e->maxxyz.x() - origin.x()) / direction.x();
    } else if (origin.x() > e->maxxyz.x() || origin.x() < e->minxyz.x())
        return false;

    if (t_xmax <= 0)
        return false;

    if (direction.y() > 1e-4) {
        t_ymin = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymax = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if (direction.y() < -1e-4) {
        t_ymax = (e->minxyz.y() - origin.y()) / direction.y();
        t_ymin = (e->maxxyz.y() - origin.y()) / direction.y();
    } else if (origin.y() > e->maxxyz.y() || origin.y() < e->minxyz.y())
        return false;

    if (t_ymax <= 0)
        return false;

    if (direction.z() > 1e-4) {
        t_zmin = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmax = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if (direction.z() < -1e-4) {
        t_zmax = (e->minxyz.z() - origin.z()) / direction.z();
        t_zmin = (e->maxxyz.z() - origin.z()) / direction.z();
    } else if (origin.z() > e->maxxyz.z() || origin.z() < e->minxyz.z())
        return false;

    if (t_zmax <= 0)
        return false;

    float t0, t1;
    t0 = max(t_zmin, max(t_xmin, t_ymin)); // 计算最大交点参数
    t1 = min(t_zmax, min(t_xmax, t_ymax)); // 计算最小交点参数

    if (t0 < t1) {
        the_t = (t0 > 0) ? t0 : t1; // 设置交点参数 the_t
        return true;
    } else {
        return false;
    }
};

bool Mesh::hit_intersect(const Ray& r, Hit& h, float tmin, BVH_TreeNode* e)
{ // 已经通过AABB包围盒测试的
    if (e->rc == nullptr && e->lc == nullptr) { // 叶子节点
        bool result = false;
        for (auto it = e->first_pel; it != e->first_pel + e->pel_num; ++it) {
            result |= ((Triangle*)triangles[(*it).index])->intersect(r, h, tmin);
        }
        return result;
    }
    float lc_t,rc_t;
    bool lc_hit = Ray_hit_AABB(e->lc, r, lc_t);
    bool rc_hit = Ray_hit_AABB(e->rc, r, rc_t);

    if (!(lc_hit || rc_hit))
        return false;

    // bool real_hit;

    if (lc_hit && rc_hit) {
        if (lc_t < rc_t) {
            return hit_intersect(r, h, tmin, e->lc) ? true : hit_intersect(r, h, tmin, e->rc);
        } else {
            return hit_intersect(r, h, tmin, e->rc) ? true : hit_intersect(r, h, tmin, e->lc);
        }
    } else if (lc_hit)
        return hit_intersect(r, h, tmin, e->lc);
    else
        return hit_intersect(r, h, tmin, e->rc);
};

bool Mesh::intersect(const Ray& r, Hit& h, float tmin)
{

    float t;
    if (!Ray_hit_AABB(root, r, t))
        return false;

    return hit_intersect(r, h, tmin, root);
}

Mesh::Mesh(const char* filename, Material* material)
    : Object3D(material)
{
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v"); // 顶点信息标识符
    std::string fTok("f"); // 顶点信息标识符
    std::string texTok("vt"); // 纹理坐标信息标识符
    std::string vnTok("vn"); // 法线向量信息标识符

    char bslash = '/', space = ' ';
    std::string tok;
    bool texture_and_normal = false;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;

        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        } else if (tok == fTok) {
            TriangleIndex vrig, trig, nrig;
            if (line.find(bslash) != std::string::npos) { // 限定两种输入方式f 2 3 4与f 2/3/4 3/4/5 4/5/6
                texture_and_normal = true;
                std::replace(line.begin(), line.end(), bslash, space);

                std::stringstream facess(line);
                facess >> tok;
                for (int ii = 0; ii < 3; ii++) {
                    facess >> vrig[ii] >> trig[ii] >> nrig[ii];
                    trig[ii]--;
                    vrig[ii]--;
                    nrig[ii]--;
                }
            } else {
                for (int ii = 0; ii < 3; ii++) {
                    ss >> vrig[ii];
                    vrig[ii]--;
                }
            }
            vIndex.push_back(vrig);
            tIndex.push_back(trig);
            nIndex.push_back(nrig);
        } else if (tok == texTok) {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
            vt.push_back(texcoord);
        } else if (tok == vnTok) {
            Vector3f vnvec;
            ss >> vnvec[0] >> vnvec[1] >> vnvec[2];
            vn.push_back(vnvec);
        }
    }
    f.close();
    computeNormal(); // 计算法线向量

    std::cout << "Normal and (possible) texture have been computed." << endl;

    computeAABB(); // 计算AABB包围盒

    std::cout << "AABB box has been computed." << endl;
    std::cout << "triangle num:" << triangles_info.size() << endl;

    // 递归建树
    root = new BVH_TreeNode(nullptr, triangles_info.begin(), (int)triangles_info.size());
    generate_child(root);

    std::cout << "BVH tree has set up." << endl;
}

// float
void Mesh::computeAABB()
{
    if ((int)triangles.size() <= 0) {
        printf("Obj File Errors.\n");
        exit(0);
    }

    for (int i = 0; i < (int)triangles.size(); i++) { // 遍历每个三角形
        float nmin[3], nmax[3]; // x y z
        for (int j = 0; j < 3; j++) {
            if (j == 0) {
                nmin[0] = nmax[0] = ((Triangle*)triangles[i])->vertices[j].x();
                nmin[1] = nmax[1] = ((Triangle*)triangles[i])->vertices[j].y();
                nmin[2] = nmax[2] = ((Triangle*)triangles[i])->vertices[j].z();
            } else {
                if (((Triangle*)triangles[i])->vertices[j].x() < nmin[0])
                    nmin[0] = ((Triangle*)triangles[i])->vertices[j].x();
                else if (((Triangle*)triangles[i])->vertices[j].x() > nmax[0])
                    nmax[0] = ((Triangle*)triangles[i])->vertices[j].x();

                if (((Triangle*)triangles[i])->vertices[j].y() < nmin[1])
                    nmin[1] = ((Triangle*)triangles[i])->vertices[j].y();
                else if (((Triangle*)triangles[i])->vertices[j].y() > nmax[1])
                    nmax[1] = ((Triangle*)triangles[i])->vertices[j].y();

                if (((Triangle*)triangles[i])->vertices[j].z() < nmin[2])
                    nmin[2] = ((Triangle*)triangles[i])->vertices[j].z();
                else if (((Triangle*)triangles[i])->vertices[j].z() > nmax[2])
                    nmax[2] = ((Triangle*)triangles[i])->vertices[j].z();
            }
        }
        Pel t = Pel(nmin[0], nmin[1], nmin[2], nmax[0], nmax[1], nmax[2], i); // 创建 Pel 结构体，存储当前三角形的边界信息和索引
        this->triangles_info.push_back(t);
    }
}

void Mesh::computeNormal()
{
    vn.resize(vIndex.size()); // 为法线向量数组 vn 分配空间，大小与顶点索引数组 vIndex 相同
    // 遍历每个三角形
    for (int triId = 0; triId < (int)vIndex.size(); ++triId) {
        TriangleIndex& triIndex = vIndex[triId]; // 获取当前三角形的顶点索引

        // 计算三角形的两个边向量
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];

        // 计算法线向量
        b = Vector3f::cross(a, b);
        vn[triId] = b / b.length();

        // 创建三角形对象并设置法线向量
        Triangle* triangle = new Triangle(v[triIndex[0]], v[triIndex[1]], v[triIndex[2]], material);
        triangle->normal = vn[triId];
        triangles.push_back(triangle);
    }
}

void Mesh::delete_node(BVH_TreeNode* e)
{
    if (!e)
        return;
    if (e->rc == nullptr && e->lc == nullptr) {
        delete e;
        return;
    }
    delete_node(e->lc);
    delete_node(e->rc);
    delete e;
    return;
}

Mesh::~Mesh()
{
    for (int i = 0; i < triangles.size(); i++)
        delete triangles[i];
    delete_node(root);
}

void Mesh::generate_child(BVH_TreeNode* e)
{
    if (e->pel_num < leaf_max_pel)
        return;
    // 根据最大和最小坐标差值来选择切分轴
    // 分别计算在 x、y、z 轴上的差值
    float dx = e->maxxyz.x() - e->minxyz.x();
    float dy = e->maxxyz.y() - e->minxyz.y();
    float dz = e->maxxyz.z() - e->minxyz.z();
    bool (*comp)(Pel x); // 指向比较函数的指针
                         // 根据差值大小选择切分轴和计算当前节点的中心点位置
    if (dx > dy && dx > dz) {
        comp = Pel_cen_x; // 按照 x 轴坐标进行比较
        now_center = e->minxyz.x() + dx / 2; // 当前节点的中心点位置
    } else if (dy > dz) {
        comp = Pel_cen_y; // 按照 y 轴坐标进行比较
        now_center = e->minxyz.y() + dy / 2;
    } else {
        comp = Pel_cen_z; // 按照 z 轴坐标进行比较
        now_center = e->minxyz.z() + dz / 2;
    }

    // 根据切分轴进行元素的划分
    auto bound = partition(e->first_pel, e->first_pel + e->pel_num, comp);
    int distance1 = distance(e->first_pel, bound);
    int distance2 = e->pel_num - distance1;
    // 创建左右子节点，并递归地对左右子节点进行进一步划分
    generate_child(new BVH_TreeNode(e, e->first_pel, distance1));
    generate_child(new BVH_TreeNode(e, bound, distance2));
}

bool Pel_min_x(Pel x, Pel y)
{
    return x.minxyz.x() < y.minxyz.x();
}

bool Pel_min_y(Pel x, Pel y)
{
    return x.minxyz.y() < y.minxyz.y();
}

bool Pel_min_z(Pel x, Pel y)
{
    return x.minxyz.z() < y.minxyz.z();
}

bool Pel_max_x(Pel x, Pel y)
{
    return x.maxxyz.x() < y.maxxyz.x();
}

bool Pel_max_y(Pel x, Pel y)
{
    return x.maxxyz.y() < y.maxxyz.y();
}

bool Pel_max_z(Pel x, Pel y)
{
    return x.maxxyz.z() < y.maxxyz.z();
}

bool Pel_cen_x(Pel x)
{
    return x.centroid.x() < now_center;
}

bool Pel_cen_y(Pel x)
{
    return x.centroid.y() < now_center;
}

bool Pel_cen_z(Pel x)
{
    return x.centroid.z() < now_center;
}