//
// Created by CONG YU on 2020/12/22.
//

#ifndef ASSIGNMENT_6_UTILS_H
#define ASSIGNMENT_6_UTILS_H

#endif //ASSIGNMENT_6_UTILS_H
#include <Eigen/Core>
#include "attributes.h"
#include "Triangle.h"


void print(const Eigen::Vector4f &x);
void print(const Eigen::Vector3f &x);
void print(const VertexAttributes &x);
void print(const std::vector<VertexAttributes> &x);
Eigen::Vector4f pixelScaleToCoordinate(int refX, int refY, int width, int height);
Eigen::Vector4f pixelToCoordinate(int x, int y, int width, int height);
VertexAttributes pixelToVertex(int x, int y, int width, int height);
std::vector<VertexAttributes> get_triangle_vertices(const std::vector<Triangle> &triangles);
std::vector<VertexAttributes> get_line_vertices(const std::vector<Triangle> &triangles);
void color_black(std::vector<VertexAttributes> &vertices);
void load_insertion_buffer_to_lines(std::vector<VertexAttributes> &buffer, std::vector<VertexAttributes> &lines);
int find_triangle(std::vector<Triangle> &triangles, Eigen::Vector4f &point);
std::pair<int,int> find_nearest_vertex(std::vector<Triangle> &triangles, Eigen::Vector4f &point);
inline float distSqr(float x1, float y1, float x2, float y2);