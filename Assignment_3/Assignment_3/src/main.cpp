////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <gif.h>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <random>

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
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
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

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////
inline double sqr(double x) {
    return x*x;
}

inline double euclideanDist(const Vector3d &u, const Vector3d &v) {
    return sqrt(sqr(u.x()-v.x()) + sqr(u.y()-v.y()) + sqr(u.z()-v.z()));
}

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
double inline ray_plane_intersect(const Vector3d &planeP, const Vector3d &planeN, const Vector3d &rayP, const Vector3d &rayD) {
    double d = -planeN.dot(planeP);
    double t = -(d+rayP.dot(planeN)) / rayD.dot(planeN);
    return t;
}

double inline det(const Vector3d &P, const Vector3d &Q) {
    return P.x()*Q.y() - Q.x()*P.y();
}

bool inline is_in_parallelogram(const Vector3d &P, const Vector3d &u, const Vector3d &v) {
    double d = det(u, v);
    double area_1 = -det(P, u)/d;
    double area_2 = det(P, v)/d;
    return 0 <= area_1 && area_1 <= 1 && 0 <= area_2 && area_2 <= 1;
}

// -1 means non exist
double inline solve_quadratic_equation(double a, double b, double c) {
    double delta = sqr(b) - 4*a*c;
    if (delta < 0) {
        return -1;
    } else {
        return (-b-sqrt(delta))/(2*a);
    }
}

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// DONE:
	//
	// Compute the intersection between the ray and the sphere
	// If the ray hits the sphere, set the result of the intersection in the
	// struct 'hit'
	auto temp = ray.origin-position;
	double t = solve_quadratic_equation(
	            ray.direction.dot(ray.direction),
	            2*temp.dot(ray.direction),
	            temp.dot(temp)-sqr(radius)
	        );
	if (t >= 0) {
	    hit.position = ray.origin + t*ray.direction;
	    hit.normal = (hit.position-position).normalized();
        hit.ray_param = t;
	    return true;
	} else {
	    return false;
	}
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// DONE
	Vector3d plane_normal = u.cross(v);
	double t = ray_plane_intersect(origin, plane_normal, ray.origin, ray.direction);
	if (t >= 0) {
	    hit.position = ray.origin + t*ray.direction;
	    int sign = plane_normal.dot(ray.direction) >= 0 ? -1 : 1;
	    hit.normal = sign * plane_normal.normalized();
	    hit.ray_param = t;
	    return is_in_parallelogram(hit.position-origin, u, v);
	} else {
	    return false;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
bool get_refract_ray(double refract_index, const Ray &incident_ray, const Intersection &intersection, Ray &refract) {
    double dot_prod = incident_ray.direction.dot(intersection.normal);
    double total_internal_reflection_index = 1-sqr(refract_index)*(1-sqr(dot_prod));
    if (total_internal_reflection_index < 0) {
        return false;
    } else {
        auto direction = refract_index*incident_ray.direction
                + intersection.normal*(refract_index*dot_prod+sqrt(total_internal_reflection_index));
        refract = {intersection.position+OFFSET*direction, direction};
        return true;
    }
}
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
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

		// DONE: Shoot a shadow ray to determine if the light should affect the intersection point
		Ray ray_to_light(hit.position + OFFSET*Li , Li);
		if (!is_light_visible(scene, ray_to_light, light)) {
            continue;
		}

		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// DONE: Specular contribution
		auto view = (ray.origin - hit.position).normalized();
		auto h = (Li+view).normalized();
		Vector3d specular = mat.specular_color * std::pow(std::max(h.dot(N), 0.0), mat.specular_exponent);

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// DONE: Compute the color of the reflected ray and add its contribution to the current point color.

    Vector3d reflection_color(0, 0, 0);
    Vector3d refraction_color(0, 0, 0);

    if (max_bounce > 0) {
        auto r = (ray.direction - 2 * (ray.direction.dot(hit.normal)) * hit.normal).normalized();
        Ray reflected_ray(hit.position + r.normalized() * OFFSET, r);
        reflection_color += mat.reflection_color.cwiseProduct(shoot_ray(scene, reflected_ray, max_bounce - 1));
    }
//
//	// DONE: Compute the color of the refracted ray and add its contribution to the current point color.
//	//       Make sure to check for total internal reflection before shooting a new ray.
//
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

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// DONE:
	//
	// Find the object in the scene that intersects the ray first
	// The function must return 'nullptr' if no object is hit, otherwise it must
	// return a pointer to the hit object, and set the parameters of the argument
	// 'hit' to their expected values.

	// naive impl, can be optimized
    Intersection intersection;
    closest_hit.ray_param = 0xfffffff;
    auto& objects = scene.objects;
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
	Intersection closet_hit;
	return find_nearest_object(scene, ray, closet_hit) == nullptr;
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

typedef std::mt19937 RNG;
Vector3d generate_random_origin(const double & radius, RNG &rng) {
    std::uniform_real_distribution<double> random_gen_radius(0, radius);
    std::uniform_real_distribution<double> random_gen_angle(0, EIGEN_PI);
    double r = random_gen_radius(rng);
    double theta = random_gen_angle(rng);
    return {r*sin(theta), r*cos(theta), 0};
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
    double scale_y = 1.0 * scene.camera.focal_length * tan(scene.camera.field_of_view / 2) ; // DONE: Stretch the pixel grid by the proper amount here
    double scale_x = 1.0 * aspect_ratio * scale_y;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    // from the sensor, and is scaled from the canonical [-1,1] in order
    // to produce the target field of view.
    Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
    Vector3d x_displacement(2.0/w*scale_x, 0, 0);
    Vector3d y_displacement(0, -2.0/h*scale_y, 0);

    for (unsigned i = 0; i < w; ++i) {
        for (unsigned j = 0; j < h; ++j) {
            // DONE: Implement depth of field
            Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

            // Prepare the ray
            Vector3d C(0, 0, 0);
            int max_bounce = 5;
            if (scene.camera.is_perspective) {
                // Perspective camera
                // DONE
                int sample = 10;
                std::random_device rd;
                std::mt19937 mt(rd());
                for (int k = 0; k < sample; k++) {
                    auto ray_origin = scene.camera.position + generate_random_origin(scene.camera.lens_radius, mt);
//                    auto ray_origin = scene.camera.position;
                    Ray sample_ray = {ray_origin, (shift-ray_origin+scene.camera.position)};
                    C += shoot_ray(scene, sample_ray, max_bounce);
                }
                C /= sample;
            } else {
                Ray ray;
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
	auto read_vec3 = [] (const json &x) {
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
			// DONE
			auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
            parallelogram->u = read_vec3(entry["u"]);
            parallelogram->v = read_vec3(entry["v"]);
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
	render_scene(scene);
	return 0;
}
