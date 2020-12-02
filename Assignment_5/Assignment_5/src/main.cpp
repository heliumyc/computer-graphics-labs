// C++ include
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"
#include "data_types.h"

#include <json.hpp>

using namespace Eigen;
using json = nlohmann::json;

// const
const double NEAR = -0.005;
const double FAR = -10000;
const double OFFSET = 1e-4;

void print(const Vector4f &x) {
    std::cout << x.x() << " " << x.y() << " " << x.z() << " " << x.w() << std::endl;
}

void print(const Vector3f &x) {
    std::cout << x.x() << " " << x.y() << " " << x.z() << std::endl;
}

void print(const VertexAttributes &x) {
    std::cout << x;
}

Scene load_scene(const std::string &filename) {
    Scene scene;

    // Load json data from scene file
    json data;
    std::ifstream in(filename);
    in >> data;

    // Helper function to read a Vector3f from a json array
    auto read_vec3 = [](const json &x) {
        return Vector3f(x[0], x[1], x[2]);
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
        if (entry["Type"] == "Mesh") {
            std::string filepath = std::string(DATA_DIR) + entry["Path"].get<std::string>();
            MeshPtr mesh = std::make_shared<Mesh>(filepath);
            mesh->material = scene.materials[entry["Material"]];
            scene.meshes.push_back(mesh);
        } else {
            ObjectPtr object = nullptr;
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
            }
            object->material = scene.materials[entry["Material"]];
            scene.objects.push_back(object);
        }
    }

    return scene;
}

inline Vector4f add_weight(const Vector3f &vec, float weight) {
    return {vec.x(), vec.y(), vec.z(), weight};
}

inline Vector3f drop_weight(const Vector4f &vec) {
    return {vec.x(), vec.y(), vec.z()};
}

std::vector<VertexAttributes> get_vertices(Scene &scene, MeshPtr &mesh) {
    std::vector<VertexAttributes> vertices;
    for (int i = 0; i < mesh->facets.rows(); ++i) {
        auto row = mesh->facets.row(i);
        Vector3f V1 = mesh->vertices.row(row.x());
        Vector3f V2 = mesh->vertices.row(row.y());
        Vector3f V3 = mesh->vertices.row(row.z());
        auto normal = ((V1 - V2).cross(V1 - V3)).normalized();
        if (normal.dot(scene.camera.gaze_direction) > 0) {
            normal *= -1;
        }
        VertexAttributes va1(V1);
        VertexAttributes va2(V2);
        VertexAttributes va3(V3);
        va1.normal = normal;
        va2.normal = normal;
        va3.normal = normal;
        vertices.push_back(va1);
        vertices.push_back(va2);
        vertices.push_back(va3);
    }
    return vertices;
}

std::vector<VertexAttributes> get_lines_vertices(Scene &scene, MeshPtr &mesh) {
    // there might be duplication, but never mind
    std::vector<VertexAttributes> vertices;
    for (int i = 0; i < mesh->facets.rows(); ++i) {
        auto row = mesh->facets.row(i);
        auto V1 = mesh->vertices.row(row.x());
        auto V2 = mesh->vertices.row(row.y());
        auto V3 = mesh->vertices.row(row.z());
        vertices.emplace_back(V1);
        vertices.emplace_back(V2);
        vertices.emplace_back(V2);
        vertices.emplace_back(V3);
        vertices.emplace_back(V3);
        vertices.emplace_back(V1);
    }
    return vertices;
}

void calculate_transform_matrix(UniformAttributes &uniform) {
    // construct M_cam
    Matrix4f M_cam_inv;
    Vector3f e = uniform.scene.camera.position;
    Vector3f w = -(uniform.scene.camera.gaze_direction.normalized());
    Vector3f u = (uniform.scene.camera.view_up.cross(w)).normalized();
    Vector3f v = (w.cross(u)).normalized();
    Matrix4f M_e;
    M_e <<
        1, 0, 0, -e.x(),
            0, 1, 0, -e.y(),
            0, 0, 1, -e.z(),
            0, 0, 0, 1;
    Matrix4f M_uvw;
    M_uvw <<
          u.x(), u.y(), u.z(), 0,
            v.x(), v.y(), v.z(), 0,
            w.x(), w.y(), w.z(), 0,
            0, 0, 0, 1;
    Matrix4f M_cam = M_uvw * M_e;

    double tan_half_angle = std::tan(uniform.scene.camera.field_of_view / 2);
    double f = FAR;
    double n = NEAR;
    double t = std::abs(n) * tan_half_angle;
    double r = uniform.image_size_ratio * t;
    double b = -t;
    double l = -r;
    // no need to construct M_vp, it is done in raster.cpp
    // construct M_orth
    Matrix4f M_orth;
    M_orth <<
           2 / (r - l), 0, 0, -(r + l) / (r - l),
            0, 2 / (t - b), 0, -(t + b) / (t - b),
            0, 0, 2 / (n - f), -(n + f) / (n - f),
            0, 0, 0, 1;

    // perspective
    Matrix4f P;
    P << n, 0, 0, 0,
            0, n, 0, 0,
            0, 0, n + f, -(f * n),
            0, 0, 1, 0;

    // get M
    uniform.M = M_orth * P * M_cam;
    uniform.M_unit_to_eye = (M_orth * P).inverse();
    uniform.M_cam = M_cam;
}

void init_uniform_attribute(UniformAttributes &uniform, Scene &scene,
                            Matrix<FrameBufferAttributes, Dynamic, Dynamic> &frameBuffer) {
    uniform.scene = scene;
    uniform.image_size_ratio = (double) frameBuffer.rows() / frameBuffer.cols(); // w/h
    calculate_transform_matrix(uniform);
    // to camera space
    uniform.lights_in_camera_space.clear();
    for (const auto &light : uniform.scene.lights) {
        uniform.lights_in_camera_space.emplace_back(drop_weight(uniform.M_cam * add_weight(light.position, 1)), light.intensity);
    }
}

void init_frame_attribute(Matrix<FrameBufferAttributes, Dynamic, Dynamic> &frame, UniformAttributes &uniform) {
    for (int i = 0; i < frame.rows(); i++) {
        for (int j = 0; j < frame.cols(); ++j) {
            frame(i,j).color.x() = uniform.scene.background_color.x()*255;
            frame(i,j).color.y() = uniform.scene.background_color.y()*255;
            frame(i,j).color.z() = uniform.scene.background_color.z()*255;
        }
    }
}

bool is_light_visible(const UniformAttributes &uniform, const Ray &ray, const Light &light) {
    // DONE: Determine if the light is visible here
//    Intersection closet_hit;
//    return find_nearest_object(scene, ray, closet_hit) == nullptr;
    for (auto &object: uniform.scene.objects) {
        Intersection intersection;
        if (object->intersect(ray, intersection)) {
            return false;
        }
    }
    return true;
}

std::function<VertexAttributes(VertexAttributes, UniformAttributes)> get_vertex_shader() {
    return [](const VertexAttributes &va, const UniformAttributes &uniform) {
        // transform into canonical volume
        VertexAttributes out;
        out.position = uniform.M * va.position;
        out.position_in_eye = uniform.M_cam * va.position;
//        float z = out.position.w();
//        out.position = out.position / z;

        // note that normal is a vector thus does not shift, only rotate
        out.normal = drop_weight(uniform.M_cam * add_weight(va.normal, 0)).normalized(); // normal only make sense in eye space
        if (out.normal.dot(Vector3f{0,0,-1}) > 0) {
            out.normal *= -1;
        }
        return out;
    };
}

std::function<FragmentAttributes(VertexAttributes, UniformAttributes)> get_fragment_shader() {
    // vertex passed in is in fact the normal calculated at that very pixel
    return [](const VertexAttributes &va, const UniformAttributes &uniform) -> FragmentAttributes {
        // transform back into camera space
        auto pos_in_camera = va.position_in_eye;

        const Material &mat = uniform.current_material;
        Vector3f ambient_color = uniform.current_material.ambient_color.array() * uniform.scene.ambient_light.array();

        Vector3f lights_color(0, 0, 0);
        const auto hit_pos = drop_weight(pos_in_camera);
        for (const auto &light : uniform.lights_in_camera_space) {
            const auto Li = (light.position - hit_pos).normalized();
            const auto &N = va.normal;

            Ray ray_to_light(hit_pos + OFFSET * Li, Li);
            if (!is_light_visible(uniform, ray_to_light, light)) {
                continue;
            }

            // diffuse
            Vector3f diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.f);

            // specular contribution
            auto view = (-hit_pos).normalized();
            auto h = (Li + view).normalized();
            Vector3f specular = mat.specular_color * std::pow(std::max(h.dot(N), 0.f), mat.specular_exponent);

            Vector3f D = light.position - hit_pos;
            lights_color += (diffuse + specular).cwiseProduct(light.intensity) / D.squaredNorm();

        }
        lights_color += mat.ambient_color;

        float red = std::min(lights_color.x(), 1.f);
        float green = std::min(lights_color.y(), 1.f);
        float blue = std::min(lights_color.z(), 1.f);
        float alpha = uniform.color.w()/0xFFu;

        FragmentAttributes out = {red, green, blue, alpha};
        out.position = va.position;

        return out;
    };
}

std::function<FragmentAttributes(VertexAttributes, UniformAttributes)> get_line_fragment_shader() {
    // vertex passed in is in fact the normal calculated at that very pixel
    return [](const VertexAttributes &va, const UniformAttributes &uniform) -> FragmentAttributes {
        FragmentAttributes out(uniform.color(0) / 255, uniform.color(1) / 255, uniform.color(2) / 255,
                               uniform.color(3) / 255);
        out.position = va.position;
        out.position.z() += OFFSET/5; // make wireframe more obvious, offset z towards camera a little bit
        return out;
    };
}

std::function<FrameBufferAttributes(FragmentAttributes, FrameBufferAttributes)> get_blending_shader() {
    return [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) -> FrameBufferAttributes {
        if (fa.position[2] < previous.depth) {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        } else {
            return previous;
        }
    };
}

int main(int argc, char *argv[]) {

    // Load scene from file
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " scene.json" << "[render option: 1|2|3]" << std::endl;
        return 1;
    }
    Scene scene = load_scene(argv[1]);
    int render_option = std::stoi(argv[2], nullptr, 10);
    if (render_option > 3 || render_option < 1) {
        render_option = 3;
    }

    // The Framebuffer storing the image rendered by the rasterizer
    Matrix<FrameBufferAttributes, Dynamic, Dynamic> frameBuffer(400, 400);
//    Matrix<FrameBufferAttributes, Dynamic, Dynamic> frameBuffer(1080, 1080);

    // Global Constants
    UniformAttributes uniform;
    init_uniform_attribute(uniform, scene, frameBuffer);
    init_frame_attribute(frameBuffer, uniform);

    // Basic rasterization program
    Program program;

    // One triangle in the center of the screen
    for (auto &mesh : scene.meshes) {
        // The vertex shader is the identity
        program.VertexShader = get_vertex_shader();
        // The fragment shader uses a fixed color
        program.FragmentShader = get_fragment_shader();
        // The blending shader converts colors between 0 and 1 to uint8
        program.BlendingShader = get_blending_shader();

        uniform.current_material = mesh->material;
        switch (render_option) {
            case 1: {
                program.FragmentShader = get_line_fragment_shader();
                uniform.color = {0, 0, 0, 255};
                auto vertices = get_lines_vertices(scene, mesh);
                rasterize_lines(program, uniform, vertices, 0.5, frameBuffer);
                break;
            }
            case 2: {
                // draw triangle
                uniform.color = {0, 0, 0, 255};
                auto vertices = get_vertices(scene, mesh);
                rasterize_triangles(program, uniform, vertices, frameBuffer);

                // draw wireframe
                program.FragmentShader = get_line_fragment_shader();
                uniform.color = {0, 0, 0, 255};
                auto line_vertices = get_lines_vertices(scene, mesh);
                rasterize_lines(program, uniform, line_vertices, 0.5, frameBuffer);
                break;
            }
            case 3: {
                auto vertices = get_vertices(scene, mesh);
                rasterize_triangles(program, uniform, vertices, frameBuffer);
                break;
            }
            default: break;
        }
    }

    std::vector<uint8_t> image;
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    return 0;
}
