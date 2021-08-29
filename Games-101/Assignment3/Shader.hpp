//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include "Texture.hpp"
#include <Eigen/Eigen>

struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f &col, const Eigen::Vector3f &nor, const Eigen::Vector2f &tc, Texture *tex, Texture *bump) : 
    color(col), normal(nor), tex_coords(tc), texture(tex), bump(bump) {}

    Eigen::Vector3f view_pos;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    Texture *texture;
    Texture *bump;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
