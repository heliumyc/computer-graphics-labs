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
    Vector3f origin;
    Vector3f direction;
    Ray() = default;
    Ray(Vector3f o, Vector3f d) : origin(std::move(o)), direction(std::move(d)) { }
};

struct Light {
    Vector3f position;
    Vector3f intensity;
    Light() = default;
    Light(Vector3f p, Vector3f i) : position(std::move(p)), intensity(std::move(i)) {}
};

struct Intersection {
    Vector3f position;
    Vector3f normal;
    double ray_param{};
};

struct Camera {
    bool is_perspective;
    Vector3f position;
    double field_of_view; // between 0 and PI
    double focal_length;
    double lens_radius; // for depth of field
    Vector3f view_up = {0,1,0};
    Vector3f gaze_direction = {0,0,-1};
};

struct Material {
    Vector3f ambient_color;
    Vector3f diffuse_color;
    Vector3f specular_color;
    double specular_exponent{}; // Also called "shininess"

    Vector3f reflection_color;
    Vector3f refraction_color;
    double refraction_index{};
};

struct Object {
    Material material;
    virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
    virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};


struct Sphere : public Object {
    Vector3f position;
    double radius;

    ~Sphere() override = default;
    bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
    Vector3f origin;
    Vector3f u;
    Vector3f v;

    ~Parallelogram() override = default;
    bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
    struct Node {
        AlignedBox3f bbox;
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
    AABBTree(const MatrixXf &V, const MatrixXi &F); // Build a BVH from an existing mesh
private:
    int buildTree(int cur, int parent, const MatrixXf &V, const MatrixXi &F, const MatrixXf &centroids, std::vector<int> &triangleIdx, int start,
                  int end, int axis);
};

struct Mesh : public Object {
    MatrixXf vertices; // n x 3 matrix (n points)
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
    Vector3f background_color;
    Vector3f ambient_light;

    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<ObjectPtr> objects;
    std::vector<MeshPtr> meshes;
};

bool intersect_box(const Ray &inv_ray, const AlignedBox3f &box);

bool intersect_triangle(const Ray &ray, const Vector3f &vertex0, const Vector3f &vertex1, const Vector3f &vertex2, Intersection &hit);

void load_off(const std::string &filename, MatrixXf &V, MatrixXi &F);

AlignedBox3f bbox_triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c);

float inline solve_quadratic_equation(float a, float b, float c);

inline float sqr(float x);

void inline get_triangle(const MatrixXf &V, const MatrixXi &F, int idx, Vector3f &A, Vector3f &B, Vector3f &C);

#endif //ASSIGNMENT5_DATA_TYPES_H
