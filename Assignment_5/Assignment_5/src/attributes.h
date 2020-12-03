#pragma once

#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <set>
#include "data_types.h"

class VertexAttributes {
public:

    explicit VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1) {
        position << x, y, z, w;
        position_in_eye << 0,0,0;
        normal << 0,0,0;
    }

    explicit VertexAttributes(Eigen::Vector3f vertex) : VertexAttributes() {
        position << vertex.x(), vertex.y(), vertex.z(), 1;
    }

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
            const VertexAttributes &a,
            const VertexAttributes &b,
            const VertexAttributes &c,
            const float alpha,
            const float beta,
            const float gamma
    ) {
        VertexAttributes r;
        r.position = alpha * a.position + beta * b.position + gamma * c.position;
        r.position_in_eye = alpha * a.position_in_eye + beta * b.position_in_eye + gamma * c.position_in_eye;
        r.normal = alpha * a.normal + beta * b.normal + gamma * c.normal;
        return r;
    }

    friend std::ostream &operator<<(std::ostream &os, const VertexAttributes &v);

    Eigen::Vector4f position;
    Eigen::Vector4f position_in_eye;
    Eigen::Vector3f normal;

    // neighbor vertex in the same facet
    std::set<int> facets_ids{};
};

class FragmentAttributes {
public:
    FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1) {
        color << r, g, b, a;
    }

    Eigen::Vector4f color;
    Eigen::Vector4f position;
};

class FrameBufferAttributes {
public:
    FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255) {
        color << r, g, b, a;
        depth = -2; // this value should be between -1 and 1, 2 is further than  the visible range
    }

    Eigen::Matrix<uint8_t, 4, 1> color;
    float depth;
};

class UniformAttributes {
public:
    Scene scene;
    std::vector<Light> lights_in_camera_space;
    double image_size_ratio; // width/height
    Eigen::Matrix4f M;
    Eigen::Matrix4f M_cam;
    Eigen::Matrix4f M_unit_to_eye;
    Vector4f color = {255, 255, 255, 255};

    // materials
    Material current_material;
};