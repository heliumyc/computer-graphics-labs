// C++ include
#include <string>
#include <vector>
#include <fstream>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"
#include "data_types.h"

#include <json.hpp>
using json = nlohmann::json;
using namespace std;

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

vector<VertexAttributes> get_vertices(Scene &scene) {
    vector<VertexAttributes> vertices;
    for (auto &mesh : scene.meshes) {
        for (int i = 0; i < mesh->facets.rows(); ++i) {
            auto row = mesh->facets.row(i);
            auto V1 = mesh->vertices.row(row.x());
            auto V2 = mesh->vertices.row(row.y());
            auto V3 = mesh->vertices.row(row.z());
            vertices.emplace_back(V1);
            vertices.emplace_back(V2);
            vertices.emplace_back(V3);
        }
    }
    return vertices;
}

std::function<VertexAttributes (VertexAttributes,UniformAttributes)> get_orthogonal_vertex_shader() {
    return [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //
        return va;
    };
}

std::function<FragmentAttributes (VertexAttributes,UniformAttributes)> get_fragment_shader() {
    return [](const VertexAttributes &va, const UniformAttributes &uniform) -> FragmentAttributes {
        return {1, 0, 0};
    };
}

std::function<FrameBufferAttributes (FragmentAttributes,FrameBufferAttributes)> get_blending_shader() {
    return [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) -> FrameBufferAttributes {
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };
}

int main(int argc, char *argv[]) {

    // Load scene from file
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " scene.json" << "[render option: 1|2|3]" << std::endl;
        return 1;
    }
    Scene scene = load_scene(argv[1]);
    int render_option = stoi(argv[2], nullptr, 10);
    if (render_option > 3 || render_option < 1) {
        render_option = 3;
    }

    // The Framebuffer storing the image rendered by the rasterizer
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(500, 500);

    // Global Constants
    UniformAttributes uniform;
    uniform.camera = scene.camera;
    uniform.lights = scene.lights;

    // Basic rasterization program
    Program program;

    // The vertex shader is the identity
    program.VertexShader = get_orthogonal_vertex_shader();

    // The fragment shader uses a fixed color
    program.FragmentShader = get_fragment_shader();

    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = get_blending_shader();

    // One triangle in the center of the screen
    auto vertices = get_vertices(scene);

    switch (render_option) {
        case 1: {
            // TODO need to use another set of vertices
            rasterize_lines(program, uniform, vertices, 1, frameBuffer);
            break;
        }
        case 2: {

            break;
        }
        case 3: {
            rasterize_triangles(program, uniform, vertices, frameBuffer);
            break;
        }
        default: rasterize_triangles(program, uniform, vertices, frameBuffer);
    }

    vector<uint8_t> image;
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    return 0;
}
