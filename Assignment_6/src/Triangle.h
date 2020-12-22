//
// Created by CONG YU on 2020/12/21.
//

#ifndef ASSIGNMENT_6_TRIANGLE_H
#define ASSIGNMENT_6_TRIANGLE_H


#include <utility>

#include "attributes.h"
#include <vector>

class Triangle {

public:
    VertexAttributes vs[3];

    Triangle(const VertexAttributes &v1, const VertexAttributes &v2, const VertexAttributes &v3);

    bool isInside(Eigen::Vector4f &vertex);

    void setColor(float R, float G, float B, float alpha);

    void shift(Eigen::Vector4f &shift);

    void shift(float x, float y);

    void rotate(float degree);

    void scale(float factor);

    Eigen::Vector4f centroid();

    bool highlight = false;

private:
    static float sign (Eigen::Vector4f &p1, Eigen::Vector4f &p2, Eigen::Vector4f &p3);
};


#endif //ASSIGNMENT_6_TRIANGLE_H
