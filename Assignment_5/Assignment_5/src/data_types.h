#include <utility>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stack>

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

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;
typedef std::shared_ptr<Mesh> MeshPtr;

struct Scene {
    Vector3d background_color;
    Vector3d ambient_light;

    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<ObjectPtr> objects;
    std::vector<MeshPtr> meshes;
};

bool intersect_box(const Ray &inv_ray, const AlignedBox3d &box);

bool intersect_triangle(const Ray &ray, const Vector3d &vertex0, const Vector3d &vertex1, const Vector3d &vertex2, Intersection &hit);

void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F);

AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c);

double inline solve_quadratic_equation(double a, double b, double c);

inline double sqr(double x);

void inline get_triangle(const MatrixXd &V, const MatrixXi &F, int idx, Vector3d &A, Vector3d &B, Vector3d &C);

#endif //ASSIGNMENT5_DATA_TYPES_H
