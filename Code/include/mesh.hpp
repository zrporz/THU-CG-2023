#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

struct Pel {
    Vector3f minxyz;    // AABB包围盒的最小顶点坐标
    Vector3f maxxyz;    // AABB包围盒的最大顶点坐标
    Vector3f centroid;  // 包围盒的中心点坐标
    int index;  // 对应的索引
    Pel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int idx) {
        minxyz = Vector3f(minx, miny, minz);
        maxxyz = Vector3f(maxx, maxy, maxz);
        centroid = (minxyz + maxxyz) * 0.5; // 计算包围盒的中心点坐标
        index = idx; // 设置索引

    }
};


struct BVH_TreeNode {
    BVH_TreeNode *parent, *lc, *rc; // 父节点、左子节点和右子节点
    Vector3f minxyz,maxxyz; // AABB包围盒的最小顶点坐标和最大顶点坐标
    int pel_num;    // 包含的Pel数量
    vector<Pel>::iterator first_pel; // 叶子节点的第一个Pel迭代器
    BVH_TreeNode(BVH_TreeNode* a, vector<Pel>::iterator it, int _num);
};

bool Ray_hit_AABB(BVH_TreeNode* e, const Ray &r, float& t); // 射线与AABB包围盒相交检测

class Mesh : public Object3D {

public:
    BVH_TreeNode* produce_child(int axis, BVH_TreeNode* now_root); //axis:0为x轴，1为y轴，2为z轴

    Mesh(const char *filename, Material *m);
    ~Mesh();
    struct TriangleIndex {
        TriangleIndex() {
            x[0] = x[1] = x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // 射线与AABB包围盒相交检测
        int x[3]{};
    };

    vector<TriangleIndex> vIndex, tIndex, nIndex;   // 顶点索引、纹理坐标索引和法线索引
    vector<Vector3f> v, vn; // 顶点坐标和法线向量
    vector<Vector2f> vt;    // 纹理坐标
    vector<Object3D *> triangles;   // 三角形对象

    bool intersect(const Ray &r, Hit &h, float tmin) override;  // 射线与网格的相交检测


    bool hit_intersect(const Ray &r, Hit &h, float tmin, BVH_TreeNode* e);  // 递归相交检测
    BVH_TreeNode* root; // BVH树的根节点
    vector<Pel> triangles_info; // Pel的信息

    void delete_node(BVH_TreeNode* e);  // 删除节点


private:

    void computeNormal();   // 计算法线向量

    void computeAABB(); // 计算法线向量

    void generate_child(BVH_TreeNode* e);

};

#endif
