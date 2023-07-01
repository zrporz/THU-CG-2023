#ifndef SCENE_PARSER_H
#define SCENE_PARSER_H

#include <cassert>
#include <vecmath.h>

class Camera;
class Light;
class Material;
class Object3D;
class Group;
class Sphere;
class Plane;
class Triangle;
class Transform;
class Mesh;
class Curve;
class RevSurface;

#define MAX_PARSER_TOKEN_LENGTH 1024

class SceneParser {
public:

    SceneParser() = delete;
    SceneParser(const char *filename);

    ~SceneParser();

    Camera *getCamera() const {
        return camera;
    }

    Vector3f getBackgroundColor() const {
        return background_color;
    }

    int getNumLights() const {
        return num_lights;
    }

    Light *getLight(int i) const {
        assert(i >= 0 && i < num_lights);
        return lights[i];
    }

    int getNumMaterials() const {
        return num_materials;
    }

    Material *getMaterial(int i) const {
        assert(i >= 0 && i < num_materials);
        return materials[i];
    }

    Group *getGroup() const {
        return group;
    }

private:

    // 初始化解析器
    void parseFile();
    // 解析摄像机
    void parsePerspectiveCamera();
    // 解析景深相机
    void parseDofCamera();
    // 解析背景
    void parseBackground();
    // 解析光照
    void parseLights();
    // 解析点光源
    Light *parsePointLight();
    // 解析方向光源
    Light *parseDirectionalLight();
    // 解析材质
    void parseMaterials();
    // 解析材质
    Material *parseMaterial();
    // 解析物体
    Object3D *parseObject(char token[MAX_PARSER_TOKEN_LENGTH]);
    // 解析组
    Group *parseGroup();
    // 解析球体
    Sphere *parseSphere();
    // 解析平面
    Plane *parsePlane();
    // 解析三角形
    Triangle *parseTriangle();
    // 解析三角形网格
    Mesh *parseTriangleMesh();
    // 解析变换
    Transform *parseTransform();
    // 解析曲线
    Curve *parseBezierCurve();
    // 解析B曲线
    Curve *parseBsplineCurve();
    // 解析反射表面
    RevSurface *parseRevSurface();

    // 获取一个token
    int getToken(char token[MAX_PARSER_TOKEN_LENGTH]);

    // 读取一个向量
    Vector3f readVector3f();

    // 读取一个浮点数
    float readFloat();
    // 读取一个整数
    int readInt();

    // 打开文件
    FILE *file;
    // 摄像机
    Camera *camera;
    // 背景颜色
    Vector3f background_color;
    // 数量
    int num_lights;
    // 每个光源的位置
    Light **lights;
    // 数量
    int num_materials;
    // 每个材质的材质
    Material **materials;
    // 每个材质的当前材质
    Material *current_material;
    // 组
    Group *group;
};

#endif // SCENE_PARSER_H
