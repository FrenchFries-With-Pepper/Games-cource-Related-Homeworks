//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

template <class T>
inline T max(T a, T b)
{
    if (a > b)
        return a;
    return b;
}

template <class T>
inline T min(T a, T b)
{
    if (a > b)
        return b;
    return a;
}

inline cv::Vec3b lerp(float t, cv::Vec3b min, cv::Vec3b max)
{
    return (1 - t) * min + t * max;
}

class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int u1 = min((int)(u_img + 0.5), width);
        int u0 = max((int)(u_img - 0.5), 0);
        int v1 = min((int)(v_img + 0.5), height);
        int v0 = max((int)(v_img - 0.5), 0);
        float s = (u_img - u0) / (u1 + 1 - u0);
        float t = (v_img - v0) / (v1 + 1 - v0);
        auto color00 = image_data.at<cv::Vec3b>(v0, u0);
        auto color01 = image_data.at<cv::Vec3b>(v1, u0);
        auto color10 = image_data.at<cv::Vec3b>(v0, u1);
        auto color11 = image_data.at<cv::Vec3b>(v1, u1);
        auto sLerped0 = lerp(s, color00, color10);
        auto sLerped1 = lerp(s, color01, color11);
        auto color = lerp(t, sLerped0, sLerped1);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
