#include <utility>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stack>
#include "myutility.h"

//
// Created by CONG YU on 2020/11/27.
//

#ifndef ASSIGNMENT5_DATA_TYPES_H
#define ASSIGNMENT5_DATA_TYPES_H

using namespace Eigen;

struct Ray {
    Vector3d origin;
    Vector3d direction;
    Ray() = default;
    Ray(Vector3d o, Vector3d d) : origin(std::move(o)), direction(std::move(d)) { }
};

struct Light {
    Vector3d position;
    Vector3d intensity;
};

struct Intersection {
    Vector3d position;
    Vector3d normal;
    double ray_param{};
};

struct Camera {
    bool is_perspective;
    Vector3d position;
    double field_of_view; // between 0 and PI
    double focal_length;
    double lens_radius; // for depth of field
};

struct Material {
    Vector3d ambient_color;
    Vector3d diffuse_color;
    Vector3d specular_color;
    double specular_exponent{}; // Also called "shininess"

    Vector3d reflection_color;
    Vector3d refraction_color;
    double refraction_index{};
};

struct Object {
    Material material;
    virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
    virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
    Vector3d position;
    double radius;

    ~Sphere() override = default;
    bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
    Vector3d origin;
    Vector3d u;
    Vector3d v;

    ~Parallelogram() override = default;
    bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
    struct Node {
        AlignedBox3d bbox;
        int parent; // Index of the parent node (-1 for root)
        int left; // Index of the left child (-1 for a leaf)
        int right; // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)

        Node(const Node& node) {
            bbox = node.bbox;
            parent = node.parent;
            left = node.left;
            right = node.right;
            triangle = node.triangle;
        }

        Node() {
            left = -1;
            right = -1;
            triangle = -1;
            parent = -1;
        }
    };

    std::vector<Node> nodes;
    int root{};

    AABBTree() = default; // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
private:
    int buildTree(int cur, int parent, const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, std::vector<int> &triangleIdx, int start,
                  int end, int axis);
};

struct Mesh : public Object {
    MatrixXd vertices; // n x 3 matrix (n points)
    MatrixXi facets; // m x 3 matrix (m triangles)

    AABBTree bvh;

    Mesh() = default; // Default empty constructor
    explicit Mesh(const std::string &filename);
    ~Mesh() override = default;
    bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
    Vector3d background_color;
    Vector3d ambient_light;

    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<ObjectPtr> objects;
};

////////////////
// utility
///////////////
// Reference: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
// the ray.direction has been inversed for performance
bool intersect_box(const Ray &inv_ray, const AlignedBox3d &box) {
    // Compute whether the ray intersects the given box.
    // There is no need to set the resulting normal and ray parameter, since
    // we are not testing with the real surface here anyway.
    Vector3d left_bottom =  box.corner(AlignedBox3d::CornerType::BottomLeftFloor);
    Vector3d right_top = box.corner(AlignedBox3d::CornerType::TopRightCeil);
    double tx1 = (left_bottom.x() - inv_ray.origin.x()) * inv_ray.direction.x();
    double tx2 = (right_top.x() - inv_ray.origin.x()) * inv_ray.direction.x();

    double tmin = std::min(tx1, tx2);
    double tmax = std::max(tx1, tx2);

    double ty1 = (left_bottom.y() - inv_ray.origin.y()) * inv_ray.direction.y();
    double ty2 = (right_top.y() - inv_ray.origin.y()) * inv_ray.direction.y();

    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    double tz1 = (left_bottom.z() - inv_ray.origin.z()) * inv_ray.direction.z();
    double tz2 = (right_top.z() - inv_ray.origin.z()) * inv_ray.direction.z();

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    if (tmin > tmax)
    {
        return false;
    }

    return true;
}

// Reference:
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool intersect_triangle(const Ray &ray, const Vector3d &vertex0, const Vector3d &vertex1, const Vector3d &vertex2, Intersection &hit) {
    /**
     * Möller–Trumbore intersection algorithm
     * from wikipedia !!!!!!!!!!
     */
    const double EPSILON = 0.0000001;
    Vector3d edge1, edge2, h, s, q;
    double a, f, u, v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = ray.direction.cross(edge2);
    a = edge1.dot(h);
    if (std::abs(a) < EPSILON) {
        return false;    // This ray is parallel to this triangle.
    }
    f = 1.0 / a;
    s = ray.origin - vertex0;
    u = f * s.dot(h);
    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * ray.direction.dot(q);
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = f * edge2.dot(q);
    if (t > EPSILON) // ray intersection
    {
        hit.position = ray.origin + t * ray.direction;
        hit.normal = edge1.cross(edge2).normalized();
        if (hit.normal.dot(ray.direction) >= 0) {
            hit.normal = -hit.normal;
        }
        hit.ray_param = t;
        return true;
    } else {
        return false;
    } // This means that there is a line intersection but not a ray intersection.
}

///////////////////
// implementation
///////////////////

Mesh::Mesh(const std::string &filename) {
    // Load a mesh from a file (assuming this is a .off file), and create a bvh
    load_off(filename, vertices, facets);
    bvh = AABBTree(vertices, facets);
}

int AABBTree::buildTree(int cur, int parent, const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, std::vector<int> &triangleIdx,
                        int start, int end, int axis) {
    if (start > end) {
        // will never happen
        return -1;
    }

    axis = axis % 3;
    // sort
    std::sort(triangleIdx.begin()+start, triangleIdx.begin()+end+1, [&](int i, int j) -> bool {
        return centroids(i, axis) < centroids(j, axis);
    });

    if (start == end) {
        // leave
        Vector3d A, B, C;
        nodes[cur].triangle = triangleIdx[start];
        nodes[cur].left = -1;
        nodes[cur].right = -1;
        nodes[cur].parent = parent;
        get_triangle(V, F, triangleIdx[start], A, B, C);
        nodes[cur].bbox = bbox_triangle(A, B, C);
    } else {
        // internal node
        int mid = start + (end - start) / 2;
        int left = buildTree(cur * 2, cur, V, F, centroids, triangleIdx, start, mid, axis + 1);
        int right = buildTree(cur * 2 + 1, cur, V, F, centroids, triangleIdx, mid + 1, end, axis + 1);
        nodes[cur].left = left;
        nodes[cur].right = right;
        nodes[cur].parent = parent;
        nodes[cur].triangle = -1;
        nodes[cur].bbox = nodes[left].bbox.merged(nodes[right].bbox);
    }
    return cur;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i) {
        for (int k = 0; k < F.cols(); ++k) {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // init nodes
    nodes.clear();
    int maxLen = 2 * (int) F.rows() - 1 + 2; // 1 for dummy node
    for (int i = 0; i < maxLen; ++i) {
        nodes.emplace_back();
    }

    // DONE (Assignment 3)
    std::vector<int> triangleIdx(centroids.rows());
    for (int i = 0; i < triangleIdx.size(); ++i) {
        triangleIdx[i] = i;
    }

    // Method (1): Top-down approach.
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    root = buildTree(1, -1, V, F, centroids, triangleIdx, 0, (int) triangleIdx.size() - 1, 0);
}

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
    // Compute the intersection between the ray and the sphere
    // If the ray hits the sphere, set the result of the intersection in the
    // struct 'hit'
    auto temp = ray.origin - position;
    double t = solve_quadratic_equation(
            ray.direction.dot(ray.direction),
            2 * temp.dot(ray.direction),
            temp.dot(temp) - sqr(radius)
    );
    if (t >= 0) {
        hit.position = ray.origin + t * ray.direction;
        hit.normal = (hit.position - position).normalized();
        hit.ray_param = t;
        return true;
    } else {
        return false;
    }
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
    Vector3d right_hand_side = this->origin - ray.origin;
    Matrix3d A;

    // initializing left hand side matrix and right hand side vector
    for (int i = 0; i < 3; ++i) {
        A(i, 0) = ray.direction(i);
        A(i, 1) = -this->u(i);
        A(i, 2) = -this->v(i);
    }

    if (A.determinant() == 0) {
        return false;
    }

    Vector3d intersection_coeffictions = A.colPivHouseholderQr().solve(right_hand_side);
    if (intersection_coeffictions(0) > 0 && intersection_coeffictions(1) >= 0 && intersection_coeffictions(2) >= 0 &&
        intersection_coeffictions(1) <= 1 && intersection_coeffictions(2) <= 1)
    {
        hit.position = ray.origin + intersection_coeffictions(0) * ray.direction;

        if (this->v.cross(this->u).dot(-ray.direction) >= 0) {
            hit.normal = this->v.cross(this->u).normalized();
        } else {
            hit.normal = this->u.cross(this->v).normalized();
        }

        hit.ray_param = intersection_coeffictions(0);
        return true;
    }
    return false;
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
    closest_hit.ray_param = INT32_MAX;
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.
    std::stack<AABBTree::Node> possible_nodes;
    possible_nodes.push(this->bvh.nodes[this->bvh.root]);
    closest_hit.ray_param = INT32_MAX;

    while(!possible_nodes.empty()) {
        AABBTree::Node curr = possible_nodes.top();
        possible_nodes.pop();
        Vector3d inverse_direction(1.0 / ray.direction[0], 1.0 / ray.direction[1], 1.0 / ray.direction[2]);
        Ray inverse_ray(ray.origin, inverse_direction);
        if (intersect_box(inverse_ray, curr.bbox)) {
            if (curr.triangle != -1) {
                // here to calculate triangle intersect with ray
                int triangle_index = curr.triangle;
                int vertex_a_index = this->facets(triangle_index, 0);
                int vertex_b_index = this->facets(triangle_index, 1);
                int vertex_c_index = this->facets(triangle_index, 2);

                Intersection hit;
                if (intersect_triangle(ray, this->vertices.row(vertex_a_index), this->vertices.row(vertex_b_index), this->vertices.row(vertex_c_index), hit)) {
                    if(hit.ray_param < closest_hit.ray_param) {
                        closest_hit = hit;
                    }
                }
            } else {
                if (curr.left != -1) {
                    possible_nodes.push(this->bvh.nodes[curr.left]);
                }
                if (curr.right != -1) {
                    possible_nodes.push(this->bvh.nodes[curr.right]);
                }
            }
        }
    }

    if (closest_hit.ray_param != INT32_MAX) {
        return true;
    }

    return false;
}

#endif //ASSIGNMENT5_DATA_TYPES_H
