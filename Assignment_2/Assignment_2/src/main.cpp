// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

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
double inline ray_plain_intersect(const Vector3d &planeP, const Vector3d &planeN, const Vector3d &rayP, const Vector3d &rayD) {
    double d = -planeN.dot(planeN);
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

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// DONE: Parameters of the parallelogram (position of the lower-left corner + two sides)
    Vector3d pgram_origin(-1, 0, 0);
	// !!! u and v only means direction, not endpoint
    Vector3d pgram_u(0.5, 0, 0.5);
    Vector3d pgram_v(0.5, 0.5, 0);

    // Single light source
	const Vector3d light_position(-0.5, 0.3, 0.3);
    Vector3d plane_normal = pgram_u.cross(pgram_v);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// DONE: Check if the ray intersects with the parallelogram
            double t = ray_plain_intersect(pgram_origin, plane_normal, ray_origin, ray_direction);
            if (t < 0) {
                // ray and plane does not intersect
                continue;
            }
            Vector3d intersect = ray_origin + t*ray_direction;
			if (is_in_parallelogram(intersect-pgram_origin, pgram_u, pgram_v)) {
				// DONE: The ray hit the parallelogram, compute the exact intersection point
				const Vector3d& ray_intersection = intersect;

				// DONE: Compute normal at the intersection point
				Vector3d ray_normal = plane_normal.normalized();
				if (ray_normal.dot(ray_direction) >= 0) {
				    ray_normal = -ray_normal;
				}

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1.5);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

    // DONE: Parameters of the parallelogram (position of the lower-left corner + two sides)
    Vector3d pgram_origin(-1, 0, 0);
    // !!! u and v only means direction, not endpoint
    Vector3d pgram_u(0.5, 0, 0.5);
    Vector3d pgram_v(0.5, 0.5, 0);

    // Single light source
    const Vector3d light_position(-0.5, 0.3, 0.3);
    Vector3d plane_normal = pgram_u.cross(pgram_v);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// DONE: Prepare the ray (origin point and direction)
			Vector3d pixel = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_origin = RowVector3d(0,0,2);
			Vector3d ray_direction = pixel - ray_origin;

            // DONE: Check if the ray intersects with the parallelogram
            double t = ray_plain_intersect(pgram_origin, plane_normal, ray_origin, ray_direction);
            if (t < 0) {
                // ray and plane does not intersect
                continue;
            }
            Vector3d intersect = ray_origin + t*ray_direction;
            if (is_in_parallelogram(intersect-pgram_origin, pgram_u, pgram_v)) {
                // DONE: The ray hit the parallelogram, compute the exact intersection point
                const Vector3d& ray_intersection = intersect;

                // DONE: Compute normal at the intersection point
                Vector3d ray_normal = plane_normal.normalized();
                if (ray_normal.dot(ray_direction) >= 0) {
                    ray_normal = -ray_normal;
                }
				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading_p_10000_mirror.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	double kDiffuse = 1.;
	double kSpecular = 1.;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	// color
	double R = 0.5;
	double G = 0.4;
	double B = 0.1;

	// specular params
	double p = 10000;

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				Vector3d illumination = light_position-ray_intersection;
				Vector3d view = ray_origin - ray_intersection;
				Vector3d h = illumination+view;

				// DONE: Add shading parameter here
				diffuse(i,j) = kDiffuse * illumination.normalized().transpose() * ray_normal;
				specular(i,j) = kSpecular * h.normalized().transpose() * ray_normal;

				// Simple diffuse model
				C(i,j) = ambient + std::max(diffuse(i,j), 0.) + std::pow(std::max(specular(i,j), 0.), p);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(R*C,G*C,B*C,A,filename);
}

int main() {
	raytrace_sphere();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();

	return 0;
}
