
#include "camera.hpp"
#include "constants.hpp"
#include "group.hpp"
#include "hit.hpp"
#include "image.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "utils.hpp"
using namespace std;
Vector3f getPtColor(Ray ray, Group* group, int depth);
class PathTracer {
    // 定义场景解析器
    const SceneParser& scene;
    // 定义输出文件
    const char* output_file;

    int pt_samples;

public:
    PathTracer(const SceneParser& scene, const char* output, int pt_samples)
        : scene(scene)
        , output_file(output), pt_samples(pt_samples) {

        };
    // 进行路径采样
    void trace()
    {
        // 获取摄像机
        Camera* camera = scene.getCamera();
        // 获取宽度和高度
        int w = camera->getWidth();
        int h = camera->getHeight();
        // 创建一个图像
        Image myImage(w, h);
        // 获取背景颜色
        Vector3f background = scene.getBackgroundColor();
        // 获取组
        Group* group = scene.getGroup();
        // 遍历每个像素
#pragma omp parallel for schedule(dynamic, 1) //openmp加速

        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {

                // 定义最终颜色
                Vector3f finalColor = Vector3f::ZERO;
                // 遍历采样次数
                for (int s = 0; s < pt_samples; s++) {

                    // 获取摄像机的角度,使用画布网格随机扰动的操作，实现抗锯齿效果
                    Vector2f ori = Vector2f(x + RND1 , y + RND1 );
                    // 生成光路
                    Ray camRay = camera->generateRay(ori);

                    finalColor += getPtColor(camRay, group, 0);
                }
                // 将采样结果归一化
                finalColor = finalColor / float(pt_samples);
                // 将采样结果转换为浮点数
                finalColor = Vector3f(toFloat(finalColor.x()), toFloat(finalColor.y()), toFloat(finalColor.z()));
                // 将采样结果存入图像
                myImage.SetPixel(x, y, finalColor);
            }
            // 如果是每5个像素输出一次，则输出一次
            if (x % 5 == 0)
                cout << "Finished:" << (float(x) * 100) / w << "%" << endl;
        }
        // 将图像保存到指定路径
        myImage.SaveImage(output_file);
    }
};
Vector3f getPtColor(Ray ray, Group* group, int depth)
{
    Vector3f color=Vector3f::ZERO;
    // Vector3f hit_color;
    Vector3f cf(1, 1, 1);
    while (true) {
        if (cf.max() < 1e-3 || depth > pt_max_depth) return color;
        Hit hit;
        if (group->intersect(ray, hit, 0)) {
            // 获取相交点的材质信息
            Material* material = hit.getMaterial();
            bool is_texture;
            Vector3f hit_color = hit.get_color(is_texture);
            if (!is_texture)
                hit_color = material->color;
            // 获取材质的自发光颜色、法线和下一条光线的起点
            color += cf * material->emission;
            cf = cf * hit_color;

            Vector3f hit_emission = material->emission;

            Vector3f N = hit.getNormal().normalized();
            Vector3f next_origin = ray.getOrigin() + hit.getT() * ray.getDirection();
            Vector3f ray_direction = ray.getDirection().normalized();

            // 根据随机数生成的类型决策来确定下一条光线的方向和类型
            float type_decision = RND2;
            float b = Vector3f::dot(ray_direction, N);
            if (type_decision < material->type.x()) { // 漫反射

                // 计算局部坐标系的基向量
                Vector3f z_ = Vector3f::cross(ray_direction, N);
                Vector3f x_ = Vector3f::cross(z_, N);
                z_.normalize();
                x_.normalize();

                // 生成漫反射的下一条光线方向
                Vector3f next_direction;
                if (b < 0)
                    next_direction = RND1 * z_ + RND1 * x_ + RND2 * N;
                else
                    next_direction = RND1 * z_ + RND1 * x_ - RND2 * N;
                next_direction.normalize();

                // 计算下一条光线的颜色
                ray = Ray(next_origin, next_direction);

            } else if (type_decision < material->type.x() + material->type.y()) { // 镜面反射
                Vector3f next_direction = ray_direction - N * (b * 2);
                next_direction.normalize();
                ray = Ray(next_origin, next_direction);

            } else { // 折射
                float n = material->refractive_index; // 折射率
                float R0 = ((1.0 - n) * (1.0 - n)) / ((1.0 + n) * (1.0 + n)); // 根据电动力学理论计算被反射光线比例
                if (b > 0) {
                    N.negate();
                } else {
                    n = 1.0 / n;
                }

                float cos1 = -Vector3f::dot(N, ray_direction); // 入射角theta余弦值
                float cos2 = 1.0 - n * n * (1.0 - cos1 * cos1); // 出射角theta2 cos(theta2)的平方
                Vector3f reflect = (ray_direction + N * (cos1 * 2));
                if (cos2 < 0) { // 全反射
                    ray = Ray(next_origin, reflect);
                } else {

                    // Schlick估计菲涅尔项，菲涅尔项（Fresnel term）是描述光线在介质边界处的反射和折射现象的数学项。它用于计算光线在界面上发生反射和折射的概率。
                    float Rprob = R0 + (1.0 - R0) * pow(1.0 - cos1, 5.0);

                    Vector3f refrac = ((ray_direction * n) + (N * (n * cos1 - sqrt(cos2)))).normalized();
                    float P = 0.25 + 0.5 * Rprob;
                    if (cos2 > 0 && RND2 > Rprob) {
                        ray = Ray(next_origin, refrac);
                    }
                    else{
                        ray = Ray(next_origin, reflect);
                    }
                }
            }
            depth += 1;
        } else {
            return color;
        }
    }
    return color;
}