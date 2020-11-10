////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>
#include <queue>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"

using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// const
constexpr double OFFSET = 1e-4;
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// define helper function
////////////////////////////////////////////////////////////////////////////////
inline double sqr(double x) {
    return x * x;
}

inline double euclideanDist(const Vector3d &u, const Vector3d &v) {
    return sqrt(sqr(u.x() - v.x()) + sqr(u.y() - v.y()) + sqr(u.z() - v.z()));
}

double inline det(const Vector3d &P, const Vector3d &Q) {
    return P.x() * Q.y() - Q.x() * P.y();
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

void inline get_triangle(const MatrixXd &V, const MatrixXi &F, int idx, Vector3d &A, Vector3d &B, Vector3d &C) {
    auto row = F.row(idx);
    A = V.row(row.x());
    B = V.row(row.y());
    C = V.row(row.z());
}

int inline largest_index(const Vector3d &vec) {
    if (vec.x() > vec.y() && vec.x() > vec.z()) {
        return 0;
    } else if (vec.y() > vec.z() && vec.y() > vec.x()) {
        return 1;
    } else {
        return 2;
    }
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
    Vector3d origin;
    Vector3d direction;

    Ray() {}

    Ray(Vector3d o, Vector3d d) : origin(o), direction(d) {}
};

struct Light {
    Vector3d position;
    Vector3d intensity;
};

struct Intersection {
    Vector3d position;
    Vector3d normal;
    double ray_param;
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
    double specular_exponent; // Also called "shininess"

    Vector3d reflection_color;
    Vector3d refraction_color;
    double refraction_index;
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

    virtual ~Sphere() = default;

    virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
    Vector3d origin;
    Vector3d u;
    Vector3d v;

    virtual ~Parallelogram() = default;

    virtual bool intersect(const Ray &ray, Intersection &hit) override;
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
		}
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default; // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
private:
    int buildTree(int cur, int parent, const MatrixXd &V, const MatrixXi &F, const MatrixXd &centroids, std::vector<int> &boxIdx, int start,
                  int end, int axis);
};

struct Mesh : public Object {
    MatrixXd vertices; // n x 3 matrix (n points)
    MatrixXi facets; // m x 3 matrix (m triangles)

    AABBTree bvh;

    Mesh() = default; // Default empty constructor
    Mesh(const std::string &filename);

    virtual ~Mesh() = default;

    virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
    Vector3d background_color;
    Vector3d ambient_light;

    Camera camera;
    std::vector<Material> materials;
    std::vector<Light> lights;
    std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
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

Mesh::Mesh(const std::string &filename) {
    // Load a mesh from a file (assuming this is a .off file), and create a bvh
    load_off(filename, vertices, facets);
    bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
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

// BVH Implementation
bool testTreeValid(int root, std::vector<AABBTree::Node> nodes) {
    if (root <= 0) {
        return true;
    }
    auto &node = nodes[root];
    auto &leftNode = nodes[node.left];
    auto &rightNode = nodes[node.right];
    return node.bbox.contains(leftNode.bbox) && node.bbox.contains(rightNode.bbox);
}
////////////////////////////////////////////////////////////////////////////////

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
    // sorting, based on longest axis
//    auto maxCorner = V.colwise().maxCoeff();
//    auto minCorner = V.colwise().minCoeff();
//    auto span = (maxCorner - minCorner).cwiseAbs();
//    int axis = largest_index(span);
    std::vector<int> triangleIdx(centroids.rows());
    for (int i = 0; i < triangleIdx.size(); ++i) {
        triangleIdx[i] = i;
    }
//    std::sort(triangleIdx.begin(), triangleIdx.end(), [&](int i, int j) -> bool {
//        return centroids(i, 0) < centroids(j, 0);
//    });

    // Method (1): Top-down approach.
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    root = buildTree(1, -1, V, F, centroids, triangleIdx, 0, (int) triangleIdx.size() - 1, 0);
    std::cout << "Test if tree is valid: " << testTreeValid(root, nodes) <<std::endl;

    // Method (2): Bottom-up approach.
    // Merge nodes 2 by 2, starting from the leaves of the forest, until only 1 tree is left.

}

////////////////////////////////////////////////////////////////////////////////

/**
 * P is the point on plane, N is the normal of plane
 * rayP is ray source, rayD is direction
 * simple geometry
 *
 * @param planeP
 * @param planeN
 * @param rayP
 * @param rayD
 * @return
 */
double inline
ray_plane_intersect(const Vector3d &planeP, const Vector3d &planeN, const Vector3d &rayP, const Vector3d &rayD) {
    return (planeP.dot(planeN) - rayP.dot(planeN)) / rayD.dot(planeN);
}

bool inline is_in_parallelogram(const Vector3d &P, const Vector3d &u, const Vector3d &v) {
    double d = det(u, v);
    double area_1 = -det(P, u) / d;
    double area_2 = det(P, v) / d;
    return 0 <= area_1 && area_1 <= 1 && 0 <= area_2 && area_2 <= 1;
}

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
    // DONE (Assignment 2)
    // DONE:
    //
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

// there are several method
// 1. u <= 1 and v <= 1
// 2. two triangle
// 3. ray-plane, test if point is in it.
bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
    // DONE (Assignment 2)
    // DONE
    Vector3d plane_normal = u.cross(v);
    double t = ray_plane_intersect(origin, plane_normal, ray.origin, ray.direction);
    if (t >= 0) {
        hit.position = ray.origin + t * ray.direction;
        int sign = plane_normal.dot(ray.direction) >= 0 ? -1 : 1;
        hit.normal = sign * plane_normal.normalized();
        hit.ray_param = t;
        return is_in_parallelogram(hit.position - origin, u, v);
    } else {
        return false;
    }
}

// -----------------------------------------------------------------------------

// Reference:
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool intersect_triangle(const Ray &ray, const Vector3d &vertex0, const Vector3d &vertex1, const Vector3d &vertex2,
                        Intersection &hit) {
    // DONE (Assignment 3)
    //
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.
    // Based on formulation induced in class
    // naive impl
//	auto AB = a-b;
//	auto AC = a-c;
//	// [a-b a-c r_d] * [u v t]^T = a-r_o
//	Matrix3d A;
//	A << AB , AC , ray.direction;
//	// u v t
//    Vector3d solution = A.colPivHouseholderQr().solve(a-ray.origin);
//    bool a_solution_exists = (A*solution).isApprox(b, 1e-6);
//    if (!a_solution_exists) {
//        return false;
//    }
//
//    double t = solution.z();
//    hit.position = ray.origin + t*ray.direction;
//    hit.normal = AB.cross(AC);
//    hit.ray_param = t;
//    return true;

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

// Reference:
// https://tavianator.com/2011/ray_box.html
bool intersect_box(const Ray &inv_ray, const AlignedBox3d &box) {

    // DONE (Assignment 3)
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

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {
//     TODO (Assignment 3)
//
//     Method (1): Traverse every triangle and return the closest hit.
//     naive impl, slowest, as benchmark
//	closest_hit.ray_param = MAXFLOAT;
//	Intersection hit;
//	bool has_intersection = false;
//    for (int i = 0; i < facets.rows(); ++i) {
//        auto row = facets.row(i);
//        auto A = vertices.row(row.x());
//        auto B = vertices.row(row.y());
//        auto C = vertices.row(row.z());
//        if (intersect_triangle(ray, A, B, C, hit)) {
//            if (hit.ray_param < closest_hit.ray_param) {
//                closest_hit.ray_param = hit.ray_param;
//                if (closest_hit.normal.dot(ray.direction) >= 0) {
//                    closest_hit.normal = -closest_hit.normal;
//                }
//            }
//            has_intersection = true;
//        }
//    }

//     Method (2): Traverse the BVH tree and test the intersection with a
//     triangles at the leaf nodes that intersects the input ray.
//     bfs
    bool has_intersection = false;
    closest_hit.ray_param = INT_MAX;
    Intersection hit;
    std::queue<int> q;
    q.push(bvh.root);

    Ray inv_ray(ray.origin, ray.direction.cwiseInverse());

    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        if (cur < 0) {
            continue;
        }
        auto &cur_node = bvh.nodes[cur];
        if (intersect_box(inv_ray, cur_node.bbox)) {
            if (cur_node.triangle >= 0) {
                // leave
                Vector3d A, B, C;
                get_triangle(vertices, facets, cur_node.triangle, A, B, C);
                if (intersect_triangle(ray, A, B, C, hit)) {
                    if (hit.ray_param < closest_hit.ray_param) {
                        closest_hit = hit;
                    }
                    has_intersection = true;
                }
            } else {
                // internal node
                q.push(cur_node.left);
                q.push(cur_node.right);
            }
        }
    }

    return has_intersection;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
bool get_refract_ray(double refract_index, const Ray &incident_ray, const Intersection &intersection, Ray &refract) {
    double dot_prod = incident_ray.direction.dot(intersection.normal);
    double total_internal_reflection_index = 1 - sqr(refract_index) * (1 - sqr(dot_prod));
    if (total_internal_reflection_index < 0) {
        return false;
    } else {
        auto direction = refract_index * incident_ray.direction
                         + intersection.normal * (refract_index * dot_prod + sqrt(total_internal_reflection_index));
        refract = {intersection.position + OFFSET * direction, direction};
        return true;
    }
}
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);

Object *find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light);

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
    // Material for hit object
    const Material &mat = obj.material;

    // Ambient light contribution
    Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector3d lights_color(0, 0, 0);
    for (const Light &light : scene.lights) {
        Vector3d Li = (light.position - hit.position).normalized();
        Vector3d N = hit.normal;

        // DONE (Assignment 2, shadow rays)
        // DONE: Shoot a shadow ray to determine if the light should affect the intersection point
        Ray ray_to_light(hit.position + OFFSET * Li, Li);
        if (!is_light_visible(scene, ray_to_light, light)) {
            continue;
        }

        // Diffuse contribution
        Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

        // DONE (Assignment 2, specular contribution)
        auto view = (ray.origin - hit.position).normalized();
        auto h = (Li + view).normalized();
        Vector3d specular = mat.specular_color * std::pow(std::max(h.dot(N), 0.0), mat.specular_exponent);

        // Attenuate lights according to the squared distance to the lights
        Vector3d D = light.position - hit.position;
        lights_color += (diffuse + specular).cwiseProduct(light.intensity) / D.squaredNorm();
    }

    // DONE (Assignment 2, reflected ray)
    // DONE: Compute the color of the reflected ray and add its contribution to the current point color.
    Vector3d reflection_color(0, 0, 0);
//    if (max_bounce > 0) {
//        auto r = (ray.direction - 2 * (ray.direction.dot(hit.normal)) * hit.normal).normalized();
//        Ray reflected_ray(hit.position + r.normalized() * OFFSET, r);
//        reflection_color += mat.reflection_color.cwiseProduct(shoot_ray(scene, reflected_ray, max_bounce - 1));
//    }

    // DONE (Assignment 2, refracted ray)
    // DONE: Compute the color of the refracted ray and add its contribution to the current point color.
    //       Make sure to check for total internal reflection before shooting a new ray.
    Vector3d refraction_color(0, 0, 0);
    // do not use refraction for this time
//    if (max_bounce > 0) {
//        Ray refracted_ray;
//        if (get_refract_ray(mat.refraction_index, ray, hit, refracted_ray)) {
//            refraction_color += mat.refraction_color.cwiseProduct(shoot_ray(scene, refracted_ray, max_bounce - 1));
//        }
//    }

    // Rendering equation
    Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

    return C;
}

// -----------------------------------------------------------------------------

Object *find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
    int closest_index = -1;
    // DONE:
    //
    // Find the object in the scene that intersects the ray first
    // The function must return 'nullptr' if no object is hit, otherwise it must
    // return a pointer to the hit object, and set the parameters of the argument
    // 'hit' to their expected values.

    // naive impl, can be optimized
    Intersection intersection;
    closest_hit.ray_param = INT_MAX;
    auto &objects = scene.objects;
    for (int i = 0; i < objects.size(); i++) {
        if (objects[i]->intersect(ray, intersection) && intersection.ray_param < closest_hit.ray_param) {
            closest_hit.position = intersection.position;
            closest_hit.normal = intersection.normal;
            closest_hit.ray_param = intersection.ray_param;
            closest_index = i;
        }
    }

    if (closest_index < 0) {
        // Return a NULL pointer
        return nullptr;
    } else {
        // Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
        return scene.objects[closest_index].get();
    }
}

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light) {
    // DONE: Determine if the light is visible here
//    Intersection closet_hit;
//    return find_nearest_object(scene, ray, closet_hit) == nullptr;
    for (auto object: scene.objects) {
        Intersection intersection;
        if (object->intersect(ray, intersection)) {
            return false;
        }
    }
    return true;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
    Intersection hit;
    if (Object *obj = find_nearest_object(scene, ray, hit)) {
        // 'obj' is not null and points to the object of the scene hit by the ray
        return ray_color(scene, ray, *obj, hit, max_bounce);
    } else {
        // 'obj' is null, we must return the background color
        return scene.background_color;
    }
}

////////////////////////////////////////////////////////////////////////////////

typedef std::mt19937 RNG;

Vector3d generate_random_origin(const double &radius, RNG &rng) {
    std::uniform_real_distribution<double> random_gen_radius(0, radius);
    std::uniform_real_distribution<double> random_gen_angle(0, EIGEN_PI);
    double r = random_gen_radius(rng);
    double theta = random_gen_angle(rng);
    return {r * sin(theta), r * cos(theta), 0};
}

void render_scene(const Scene &scene) {
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    double scale_y = 1.0 * scene.camera.focal_length *
                     tan(scene.camera.field_of_view / 2); // DONE: Stretch the pixel grid by the proper amount here
    double scale_x = 1.0 * aspect_ratio * scale_y;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    // from the sensor, and is scaled from the canonical [-1,1] in order
    // to produce the target field of view.
    Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
    Vector3d x_displacement(2.0 / w * scale_x, 0, 0);
    Vector3d y_displacement(0, -2.0 / h * scale_y, 0);

    for (unsigned i = 0; i < w; ++i) {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
        for (unsigned j = 0; j < h; ++j) {
            // DONE (Assignment 2, depth of field)
            Vector3d shift = grid_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Ray ray;

            Vector3d C(0, 0, 0);
            int max_bounce = 5;
            if (scene.camera.is_perspective) {
                // Perspective camera
                // DONE (Assignment 2, perspective camera)
                int sample = 10;
                std::random_device rd;
                std::mt19937 mt(rd());
                for (int k = 0; k < sample; k++) {
                    auto ray_origin = scene.camera.position + generate_random_origin(scene.camera.lens_radius, mt);
                    Ray sample_ray = {ray_origin, (shift - ray_origin + scene.camera.position)};
                    C += shoot_ray(scene, sample_ray, max_bounce);
                }
                C /= sample;
            } else {
                // Orthographic camera
                ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
                ray.direction = Vector3d(0, 0, -1);
                C = shoot_ray(scene, ray, max_bounce);
            }

            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = 1;
        }
    }

    std::cout << "Ray tracing: 100%  " << std::endl;

    // Save to png
    const std::string filename("raytrace.png");
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
    Scene scene;

    // Load json data from scene file
    json data;
    std::ifstream in(filename);
    in >> data;

    // Helper function to read a Vector3d from a json array
    auto read_vec3 = [](const json &x) {
        return Vector3d(x[0], x[1], x[2]);
    };

    // Read scene info
    scene.background_color = read_vec3(data["Scene"]["Background"]);
    scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

    // Read camera info
    scene.camera.is_perspective = data["Camera"]["IsPerspective"];
    scene.camera.position = read_vec3(data["Camera"]["Position"]);
    scene.camera.field_of_view = data["Camera"]["FieldOfView"];
    scene.camera.focal_length = data["Camera"]["FocalLength"];
    scene.camera.lens_radius = data["Camera"]["LensRadius"];

    // Read materials
    for (const auto &entry : data["Materials"]) {
        Material mat;
        mat.ambient_color = read_vec3(entry["Ambient"]);
        mat.diffuse_color = read_vec3(entry["Diffuse"]);
        mat.specular_color = read_vec3(entry["Specular"]);
        mat.reflection_color = read_vec3(entry["Mirror"]);
        mat.refraction_color = read_vec3(entry["Refraction"]);
        mat.refraction_index = entry["RefractionIndex"];
        mat.specular_exponent = entry["Shininess"];
        scene.materials.push_back(mat);
    }

    // Read lights
    for (const auto &entry : data["Lights"]) {
        Light light;
        light.position = read_vec3(entry["Position"]);
        light.intensity = read_vec3(entry["Color"]);
        scene.lights.push_back(light);
    }

    // Read objects
    for (const auto &entry : data["Objects"]) {
        ObjectPtr object;
        if (entry["Type"] == "Sphere") {
            auto sphere = std::make_shared<Sphere>();
            sphere->position = read_vec3(entry["Position"]);
            sphere->radius = entry["Radius"];
            object = sphere;
        } else if (entry["Type"] == "Parallelogram") {
            auto parallelogram = std::make_shared<Parallelogram>();
            parallelogram->origin = read_vec3(entry["Origin"]);
            parallelogram->u = read_vec3(entry["u"]);
            parallelogram->v = read_vec3(entry["v"]);
        } else if (entry["Type"] == "Mesh") {
            // Load mesh from a file
            std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
            object = std::make_shared<Mesh>(filename);
        }
        object->material = scene.materials[entry["Material"]];
        scene.objects.push_back(object);
    }

    return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
        return 1;
    }
    Scene scene = load_scene(argv[1]);
    // test mesh
    render_scene(scene);

    return 0;
}
