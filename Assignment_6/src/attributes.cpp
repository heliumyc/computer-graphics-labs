//
// Created by CONG YU on 2020/12/21.
//
#include "attributes.h"

std::ostream &operator<<(std::ostream &os, const VertexAttributes &v) {
    os << "("
       << v.position[0] << ", "
       << v.position[1] << ", "
       << v.position[2] << ", "
       << v.position[3] << ")"
       << " color: ("
       << v.color.x() << ", "
       << v.color.y() << ", "
       << v.color.z() << ", "
       << v.color.w() << ")";
    return os;
}

UniformAttributes::UniformAttributes() {
    viewM = Eigen::Matrix<float,4,4>::Identity();
    inverseM = Eigen::Matrix<float,4,4>::Identity();
}
