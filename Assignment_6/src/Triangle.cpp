//
// Created by CONG YU on 2020/12/21.
//

#include "Triangle.h"
#include <iostream>
#include <math.h>
#include "mytutils.h"

const float PI = 3.1415926;

Triangle::Triangle(const VertexAttributes &v1, const VertexAttributes &v2, const VertexAttributes &v3) {
    vs[0] = v1;
    vs[1] = v2;
    vs[2] = v3;
}

void Triangle::setColor(float R, float G, float B, float alpha) {
}

// https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
bool Triangle::isInside(Eigen::Vector4f &pt) {
    float d1, d2, d3;
    bool has_neg, has_pos;

    d1 = sign(pt, vs[0].position, vs[1].position);
    d2 = sign(pt, vs[1].position, vs[2].position);
    d3 = sign(pt, vs[2].position, vs[0].position);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

float Triangle::sign(Eigen::Vector4f &p1, Eigen::Vector4f &p2, Eigen::Vector4f &p3) {
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

void Triangle::shift(Eigen::Vector4f &shift) {
    for (auto &x: vs) {
        x.position.x() += shift.x();
        x.position.y() += shift.y();
    }
}

void Triangle::shift(float x, float y) {
    for (auto &v: vs) {
        v.position.x() += x;
        v.position.y() += y;
    }
}

Eigen::Vector4f Triangle::centroid() {
    float mx,my;
    mx = my = 0;
    for (auto &v: vs) {
        mx += v.position.x();
        my += v.position.y();
    }
    return {mx/3.0, my/3.0, 0, 1};
}

void Triangle::rotate(float degree) {
    // move to origin and rotate
    auto center = centroid();
    float rotCos = std::cos(PI*degree/180);
    float rotSin = std::sin(PI*degree/180);
    for (auto &v: vs) {
        auto ov = v.position-center;
        float nx = ov.x()*rotCos - ov.y()*rotSin;
        float ny = ov.x()*rotSin + ov.y()*rotCos;
        v.position.x() = nx + center.x();
        v.position.y() = ny + center.y();
    }
}

void Triangle::scale(float factor) {
    // move to origin and scale
    auto center = centroid();
    for (auto &v: vs) {
        auto ov = v.position-center;
        float nx = ov.x()*factor;
        float ny = ov.y()*factor;
        v.position.x() = nx + center.x();
        v.position.y() = ny + center.y();
    }
}
