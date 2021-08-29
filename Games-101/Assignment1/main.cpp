#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radiance = M_PI / 180.F;
    float cosTheta = cos(rotation_angle * radiance);
    float sinTheta = sin(rotation_angle * radiance);
    model << cosTheta, -sinTheta, 0.f, 0.f,
        sinTheta, cosTheta, 0.f, 0.f,
        0.f, 0.f, 1.0f, 0.f,
        0.f, 0.f, 0.f, 1.f;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Eigen::Matrix4f rota_around_arbitary_vector(Eigen::Vector3f axis, float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float radiance = M_PI / 180.f;
    float cosTheta = cos(rotation_angle * radiance);
    float sinTheta = sin(rotation_angle * radiance);
    model << axis(0) * axis(0) * (1 - cosTheta) + cosTheta,
        axis(0) * axis(1) * (1 - cosTheta) + axis(2) * sinTheta,
        axis(0) * axis(2) * (1 - cosTheta) - axis(1) * sinTheta,
        axis(0) * axis(1) * (1 - cosTheta) - axis(2) * sinTheta,
        axis(1) * axis(1) * (1 - cosTheta) + cosTheta,
        axis(1) * axis(2) * (1 - cosTheta) + axis(2) * sinTheta,
        axis(0) * axis(2) * (1 - cosTheta) + axis(1) * sinTheta,
        axis(1) * axis(2) * (1 - cosTheta) - axis(2) * sinTheta,
        axis(2) * axis(2) * (1 - cosTheta) + cosTheta;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    float theta = eye_fov * M_PI / 360.f;
    float xyScale = tan(theta);
    Eigen::Matrix4f projection;
    projection << 1 / (aspect_ratio * xyScale), 0, 0, 0,
        0, 1 / xyScale, 0, 0,
        0, 0, (zNear + zFar) / (zNear - zFar), -2 * zNear * zFar / (zNear - zFar),
        0, 0, 1, 0;
    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }
    return 0;
}
