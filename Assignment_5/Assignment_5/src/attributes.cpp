//
// Created by CONG YU on 2020/11/29.
//

#include "attributes.h"

std::ostream& operator<<(std::ostream& os, const VertexAttributes &v)
{
    os << "("
            << v.position[0] << ", "
            << v.position[1] << ", "
            << v.position[2] << ", "
            << v.position[3] << ")"
            << std::endl;
    return os;
}