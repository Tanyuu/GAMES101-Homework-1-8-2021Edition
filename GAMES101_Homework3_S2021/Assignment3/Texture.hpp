//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
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
        u = u*width;
        v = (1-v)*height;
        int l = u;
        float s = u-l;
        int d = v;
        float t = v-d;

        auto tmp = image_data.at<cv::Vec3b>(d,l);
        Eigen::Vector3f u00(tmp[0], tmp[1], tmp[2]);
        tmp = image_data.at<cv::Vec3b>(d,std::min(l+1,width));
        Eigen::Vector3f u10(tmp[0], tmp[1], tmp[2]);
        tmp = image_data.at<cv::Vec3b>(std::min(d+1,height),l);
        Eigen::Vector3f u01(tmp[0], tmp[1], tmp[2]);
        tmp = image_data.at<cv::Vec3b>(std::min(d+1,height),std::min(l+1,width));
        Eigen::Vector3f u11(tmp[0], tmp[1], tmp[2]);

        Eigen::Vector3f u0 = u00 + s*(u10-u00);
        Eigen::Vector3f u1 = u01 + s*(u11-u01);
        return u0 + t*(u1-u0);
    }
};
#endif //RASTERIZER_TEXTURE_H
