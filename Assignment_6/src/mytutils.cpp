//
// Created by CONG YU on 2020/12/22.
//

#include "mytutils.h"
#include <iostream>
#include <vector>

void print(const Eigen::Vector4f &x) {
    std::cout << x.x() << " " << x.y() << " " << x.z() << " " << x.w() << std::endl;
}

void print(const Eigen::Vector3f &x) {
    std::cout << x.x() << " " << x.y() << " " << x.z() << std::endl;
}

void print(const VertexAttributes &x) {
    std::cout << x << std::endl;
}

void print(const std::vector<VertexAttributes> &x) {
    for (auto &y: x) {
        print(y);
    }
}

Eigen::Vector4f pixelScaleToCoordinate(int refX, int refY, int width, int height) {
    return {float(refX)/float(width)*2, float(-refY)/float(height)*2, 0, 1};
}

Eigen::Vector4f pixelToCoordinate(int x, int y, int width, int height) {
    return {float(x)/float(width)*2 - 1, float(height-1-y)/float(height)*2 - 1, 0, 1};
}

VertexAttributes pixelToVertex(int x, int y, int width, int height) {
    return {float(x)/float(width)*2 - 1, float(height-1-y)/float(height)*2 - 1, 0, 1};
}

std::vector<VertexAttributes> get_triangle_vertices(const std::vector<Triangle> &triangles) {
    std::vector<VertexAttributes> vertices;
    vertices.reserve(triangles.size() * 3);
    for (auto &triangle : triangles) {
        for (auto &v: triangle.vs) {
            VertexAttributes temp = v;
            if (triangle.highlight) {
                temp.color.x() = 1 - temp.color.x();
                temp.color.y() = 1 - temp.color.y();
                temp.color.z() = 1 - temp.color.z();
            }
            vertices.push_back(temp);
        }
    }
    return vertices;
}

std::vector<VertexAttributes> get_line_vertices(const std::vector<Triangle> &triangles) {
    std::vector<VertexAttributes> vertices;
    vertices.reserve(triangles.size() * 6 + 6); // 6 is reserved for possible insertion buffer elements
    for (auto &triangle : triangles) {
        for (int i = 0; i < 3; i++) {
            vertices.push_back(triangle.vs[i]);
            vertices.push_back(triangle.vs[(i+1)%3]);
        }
    }
    return vertices;
}

void color_black(std::vector<VertexAttributes> &vertices) {
    for (auto &v: vertices) {
        v.color << 0,0,0,1;
    }
}

void load_insertion_buffer_to_lines(std::vector<VertexAttributes> &buffer, std::vector<VertexAttributes> &lines) {
    if (buffer.size() == 2) {
        lines.push_back(buffer[0]);
        lines.push_back(buffer[1]);
    } else if (buffer.size() == 3) {
        lines.push_back(buffer[0]);
        lines.push_back(buffer[1]);
        lines.push_back(buffer[1]);
        lines.push_back(buffer[2]);
        lines.push_back(buffer[2]);
        lines.push_back(buffer[0]);
    }
}

int find_triangle(std::vector<Triangle> &triangles, Eigen::Vector4f &point) {
    for (int i=0; i < triangles.size(); i++) {
        auto &triangle = triangles[i];
        if (triangle.isInside(point)) {
            return i;
        }
    }
    return -1;
}

inline float sqr(float x) {
    return x*x;
}

inline float distSqr(float x1, float y1, float x2, float y2) {
    return sqr(x1-x2) + sqr(y1-y2);
}

std::pair<int,int> find_nearest_vertex(std::vector<Triangle> &triangles, Eigen::Vector4f &point) {
    int tidx = 0;
    int vidx = 0;
    float minDist = 10000000000.f;
    for (int i = 0 ; i < triangles.size();i++) {
        auto &triangle = triangles[i];
        for (int j = 0 ; j < 3; j++) {
            auto &v = triangle.vs[j];
            float d = distSqr(v.position.x(), v.position.y(), point.x(), point.y());
            if (d < minDist) {
                minDist = d;
                tidx = i;
                vidx = j;
            }
        }
    }
    return std::make_pair(tidx, vidx);
}


