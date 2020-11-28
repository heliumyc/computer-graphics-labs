#include <utility>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stack>
//
// Created by CONG YU on 2020/11/28.
//

#ifndef ASSIGNMENT5_MYUTILITY_H
#define ASSIGNMENT5_MYUTILITY_H

using namespace Eigen;

inline double sqr(double x) {
    return x * x;
}

void inline get_triangle(const MatrixXd &V, const MatrixXi &F, int idx, Vector3d &A, Vector3d &B, Vector3d &C) {
    auto row = F.row(idx);
    A = V.row(row.x());
    B = V.row(row.y());
    C = V.row(row.z());
}

void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
    std::ifstream in(filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    V.resize(nv, 3);
    F.resize(nf, 3);
    for (int i = 0; i < nv; ++i) {
        in >> V(i, 0) >> V(i, 1) >> V(i, 2);
    }
    for (int i = 0; i < nf; ++i) {
        int s;
        in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
        assert(s == 3);
    }
}

// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

// -1 means non exist
double inline solve_quadratic_equation(double a, double b, double c) {
    double delta = sqr(b) - 4 * a * c;
    if (delta < 0) {
        return -1;
    } else {
        return (-b - sqrt(delta)) / (2 * a);
    }
}


#endif //ASSIGNMENT5_MYUTILITY_H
