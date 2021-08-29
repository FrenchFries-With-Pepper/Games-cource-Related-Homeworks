//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f *v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    float centorX = float(x);
    float centorY = float(y);
    Eigen::Vector3f p(centorX, centorY, 0.f);
    bool sign[3];
    for (int i = 0; i < 3; i++)
    {
        int pNext = (i + 1) % 3;
        Eigen::Vector3f v1 = v[pNext] - v[i];
        Eigen::Vector3f v2 = p - v[i];
        Eigen::Vector3f result = v1.cross(v2);
        sign[i] = result(2) > 0.f;
    }
    if (sign[0] == sign[1] && sign[1] == sign[2])
    {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        //Homogeneous division
        for (auto &vec : v)
        {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto &vert : v)
        {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t)
{
    auto v = t.toVector4();
    float minNum = std::numeric_limits<float>::lowest();
    float maxNum = std::numeric_limits<float>::max();
    float minY = t.v[0](1);
    float minX = t.v[0](0);
    float maxX = t.v[0](0);
    float maxY = t.v[0](1);
    //find out bounding box
    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3f vectex = t.v[i];
        if (vectex(0) > maxX)
        {
            maxX = vectex(0);
        }
        if (vectex(0) < minX)
        {
            minX = vectex(0);
        }
        if (vectex(1) > maxY)
        {
            maxY = vectex(1);
        }
        if (vectex(1) < minY)
        {
            minY = vectex(1);
        }
    }

    for (int x = minX; x < maxX; x++)
    {
        for (int y = minY; y < maxY; y++)
        {
            int pixelIndex = y * width + x;
            bool frame_buffer_Changed = false;
            Eigen::Vector3f color = {0.f, 0.f, 0.f};
            Eigen::Vector3f TriangleColor={0.f,0.f,0.f};
            for (int i = 0; i < 4; i++)
            {

                float tempx = x + (0.25f + 0.5f * (i % 2));
                float tempy = y + (0.25f + 0.5f * (i / 2));
                if (insideTriangle(tempx, tempy, t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(tempx, tempy, t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < depth_buf[pixelIndex * 4 + i])
                    {
                        depth_buf[pixelIndex * 4 + i] = z_interpolated;
                        color += t.getColor();
                        TriangleColor=t.getColor();
                        frame_buffer_Changed = true;
                    }
                    else
                        color += last_buf[pixelIndex];
                }
                else
                    color += last_buf[pixelIndex];
            }
            if (frame_buffer_Changed)
            {
                last_buf[pixelIndex]=TriangleColor;
                frame_buf[pixelIndex] = color / 4.f;
            }
        }
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(last_buf.begin(),last_buf.end(),Eigen::Vector3f{0,0,0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h * 4);
    last_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
