#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sys/time.h>

#include "camera.hpp"
#include "group.hpp"
#include "image.hpp"
#include "light.hpp"
#include "scene_parser.hpp"
#include "pt.hpp"
#include "constants.hpp"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
    
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 5) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string method = argv[1];
    string inputFile = argv[2]; // only bmp is allowed.
    string outputFile = argv[3]; // only bmp is allowed.
    int pt_samples = atoi(argv[4]);
    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.

    struct timeval start, end;
    gettimeofday(&start, NULL);

    SceneParser sceneParser(inputFile.c_str());
    if (method == "pt") {   //使用pt算法光线追踪
        printf("Method: path tracing\n");
        PathTracer path_tracer(sceneParser, argv[3],pt_samples);
        path_tracer.trace();
    
    } else {    
        Camera* camera = sceneParser.getCamera();
        int width = camera->getWidth(), height = camera->getHeight();
        Image out(width, height);
        // 循环屏幕空间的像素
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) { // 计算当前像素(x,y)处相机出射光线camRay
                Ray camRay = camera->generateRay(Vector2f(x, y));
                Group* baseGroup = sceneParser.getGroup();
                Hit hit;
                // 判断camRay是否和场景有交点,并返回最近交点的数据,存储在hit中
                bool isIntersect = baseGroup->intersect(camRay, hit, 0);
                if (isIntersect) {
                    Vector3f finalColor = Vector3f::ZERO;
                    // 找到交点之后,累加来自所有光源的光强影响
                    for (int li = 0; li < sceneParser.getNumLights(); ++li) {
                        Light* light = sceneParser.getLight(li);
                        Vector3f L, lightColor;

                        light->getIllumination(camRay.pointAtParameter(hit.getT()), L, lightColor);
                        // 计算局部光强
                        finalColor += hit.getMaterial()->Shade(camRay, hit, L, lightColor);
                    }
                    out.SetPixel(x, y, finalColor);
                } else {
                    // 不存在交点,返回背景色
                    out.SetPixel(x, y, sceneParser.getBackgroundColor());
                }
            }
        }
    }
    gettimeofday(&end, NULL);
    time_t sec = end.tv_sec - start.tv_sec;
    time_t minute = sec / 60;
    cout << "Total time: " << sec << "seconds, or " << minute << "mins" << endl;
    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}
