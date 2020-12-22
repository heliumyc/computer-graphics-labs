#include "SDLViewer.h"

#include <Eigen/Core>

#include <functional>
#include <iostream>
#include <thread>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"
#include "Triangle.h"
#include "mytutils.h"
#include <chrono>
#include <thread>

Eigen::Vector4f canvas_to_unit(Eigen::Vector4f &canvas, int width, int height) {
    return {canvas.x() / float(width) * 2 - 1, (float(height)-canvas.y())/float(height)*2 -1 , 0, 1};
}

Eigen::Vector4f unit_to_canvas(Eigen::Vector4f &unit, int width, int height) {
    return {0.5*(1+unit.x())*float(width), float(height) - 0.5*(1+unit.y())*float(height), 0, 1};
}

void inverse_view_trans(UniformAttributes &uniform, float &nx, float &ny) {
    // THIS IS ONLY FOR CURSOR, NOT A SPATIAL TRANSFORMATION
    Eigen::Vector4f vec(nx, ny, 0, 1);
    vec = canvas_to_unit(vec, uniform.width, uniform.height);
    vec = uniform.inverseM*vec;
    vec = unit_to_canvas(vec, uniform.width, uniform.height);
    nx = vec.x();
    ny = vec.y();
}

void inverse_view_scale(UniformAttributes &uniform, float &relx, float &rely) {
    Eigen::Vector4f vec(relx, rely, 0, 1);
//    vec = canvas_to_unit(vec, uniform.width, uniform.height);
    vec.x() = vec.x() * uniform.inverseM(0,0);
    vec.y() = vec.y() * uniform.inverseM(1,1);
//    vec = unit_to_canvas(vec, uniform.width, uniform.height);
    relx = vec.x();
    rely = vec.y();
}

std::function<void(int, int, int, int)> get_insertion_mouse_move(SDLViewer &viewer) {
    return [&](int x, int y, int xrel, int yrel) {
        auto nx = float(x);
        auto ny = float(y);
        inverse_view_trans(viewer.uniform, nx, ny);

        while (viewer.insertionBuffer.size() > viewer.clickCount && !viewer.insertionBuffer.empty()) {
            viewer.insertionBuffer.pop_back();
        }
        viewer.insertionBuffer.emplace_back(nx, ny, 0, 1);
        viewer.redraw_next = true;
    };
}

std::function<void(int, int, bool, int, int)> get_insertion_mouse_press(SDLViewer &viewer) {
    return [&](int x, int y, bool is_pressed, int button, int clicks) {
        auto nx = float(x);
        auto ny = float(y);
        inverse_view_trans(viewer.uniform, nx, ny);

        if (is_pressed) {
            viewer.clickCount += 1;
            viewer.insertionBuffer.emplace_back(nx, ny, 0, 1);
            if (viewer.clickCount == 3) {
                // insert triangle into triangles list
                viewer.triangles.emplace_back(viewer.insertionBuffer[0], viewer.insertionBuffer[1],
                                              viewer.insertionBuffer[2]);
                viewer.insertionBuffer.clear();
                viewer.clickCount = 0;
                viewer.redraw_next = true;
            }
        }

    };
}

std::function<void(int, int, int, int)> get_translation_mouse_move(SDLViewer &viewer) {
    return [&](int x, int y, int xrel, int yrel) {
        auto nxrel = float(xrel);
        auto nyrel = float(yrel);
        inverse_view_scale(viewer.uniform, nxrel, nyrel);

//        Eigen::Vector4f shift = pixelScaleToCoordinate(xrel, yrel, viewer.width, viewer.height);
//        auto target = pixelToCoordinate(x, y, viewer.width, viewer.height);
//        std::cout << target.x() << " " << target.y() << " " << shift.x() << " " << shift.y() << std::endl;
        if (viewer.enableDrag && viewer.triangleIdxClicked >= 0) {
            auto &triangle = viewer.triangles[viewer.triangleIdxClicked];
            triangle.shift(float(nxrel), float(nyrel));
        }
        viewer.redraw_next = true;
    };
}

std::function<void(int, int, bool, int, int)> get_translation_mouse_press(SDLViewer &viewer) {
    return [&](int x, int y, bool is_pressed, int button, int clicks) {
        // click part
        auto nx = float(x);
        auto ny = float(y);
        inverse_view_trans(viewer.uniform, nx, ny);
//        std::cout << x << " " << y << " " << nx << " " << ny << std::endl;

//        print(viewer.triangles[0].vs[0]);
//        print(viewer.triangles[0].vs[1]);
//        print(viewer.triangles[0].vs[2]);

        if (is_pressed) {
            // find the triangle clicked
            Eigen::Vector4f pt(nx, ny, 0, 1);
            int index = find_triangle(viewer.triangles, pt);
            if (index >= 0) {
                // same triangle
                viewer.triangleIdxClicked = index;
                auto &triangle = viewer.triangles[viewer.triangleIdxClicked];
                triangle.highlight = true;
                viewer.enableDrag = true;
            } else {
                // clear highlight
                viewer.enableDrag = false;
                viewer.clearHighlight();
            }
        } else {
            viewer.enableDrag = false;
            viewer.clearHighlight();
        }
        viewer.redraw_next = true;
    };
}

std::function<void(int, int, bool, int, int)> get_remove_mouse_press(SDLViewer &viewer) {
    return [&](int x, int y, bool is_pressed, int button, int clicks) {
        if (is_pressed) {
            auto nx = float(x);
            auto ny = float(y);
            inverse_view_trans(viewer.uniform, nx, ny);

            Eigen::Vector4f pt(nx, ny, 0, 1);
            int index = find_triangle(viewer.triangles, pt);
            viewer.triangleIdxClicked = index;
            if (index >= 0) {
                viewer.triangles.erase(viewer.triangles.begin() + index);
            }
            viewer.redraw_next = true;
        }
    };
}

std::function<void(int, int, bool, int, int)> get_color_mouse_press(SDLViewer &viewer) {
    return [&](int x, int y, bool is_pressed, int button, int clicks) {
        if (is_pressed) {
            auto nx = float(x);
            auto ny = float(y);
            inverse_view_trans(viewer.uniform, nx, ny);

            if (!viewer.triangles.empty()) {
                // find the nearest vertex
                Eigen::Vector4f pt(nx, ny, 0, 1);
                auto pair = find_nearest_vertex(viewer.triangles, pt);
                viewer.colorTriIdx = pair.first;
                viewer.colorVertexIdx = pair.second;
                viewer.redraw_next = true;
            }
        }
    };
}

void rotate_target_triangle(SDLViewer &viewer, float degree) {
    if (degree == 0) return;
    if (viewer.triangleIdxClicked >= 0) {
        auto &t = viewer.triangles[viewer.triangleIdxClicked];
        t.rotate(degree);
    }
}

void scale_target_triangle(SDLViewer &viewer, float factor) {
    if (factor == 0) return;
    if (viewer.triangleIdxClicked >= 0) {
        auto &t = viewer.triangles[viewer.triangleIdxClicked];
        t.scale(factor);
    }
}

void set_zoom_uniform_matrix(UniformAttributes &uniform, int zoomIn) {
    float factor = zoomIn > 0? 1.2: 0.8;
    Eigen::Matrix<float, 4, 4> m;
    m << factor,0,0,0,
    0,factor,0,0,
    0,0,1,0 ,
    0,0,0,1;
    Eigen::Matrix<float, 4, 4> inverse;
    inverse << 1/factor,0,0,0 ,
    0,1/factor,0,0 ,
    0,0,1,0 ,
    0,0,0,1;
    uniform.viewM = m*uniform.viewM;
    uniform.inverseM = uniform.inverseM*inverse;
//    std::cout << uniform.viewM << std::endl;
//    std::cout << uniform.inverseM << std::endl;
}

void set_translate_uniform_matrix(UniformAttributes &uniform, float offsetX, float offsetY) {
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,offsetX,
            0,1,0,offsetY,
            0,0,1,0 ,
            0,0,0,1;
    Eigen::Matrix<float, 4, 4> inverse;
    inverse << 1,0,0,-offsetX ,
            0,1,0,-offsetY,
            0,0,1,0 ,
            0,0,0,1;
    uniform.viewM = m*uniform.viewM;
    uniform.inverseM = uniform.inverseM*inverse;
//    std::cout << uniform.viewM << std::endl;
//    std::cout << uniform.inverseM << std::endl;
}

void color_vertex(int triangleIdx, int vertexIdx, char colorChar, SDLViewer &viewer) {
    if (triangleIdx >= 0 && triangleIdx < viewer.triangles.size() && vertexIdx >= 0 && vertexIdx < 3) {
        int color = colorChar - '0';
        // some random color
        float r = 0.3f * float(color/3);
        float g = 0.3f * float(color%3);
        float b = 1-(r+g)/2;
        auto &v = viewer.triangles[triangleIdx].vs[vertexIdx];
        v.color.x() = r;
        v.color.y() = g;
        v.color.z() = b;
    }
}

void draw(Program &program, FrameBuffer& frameBuffer, SDLViewer &viewer, std::vector<Triangle> &triangles) {
    // Clear the framebuffer
    for (unsigned i = 0; i < frameBuffer.rows(); i++) {
        for (unsigned j = 0; j < frameBuffer.cols(); j++) {
            frameBuffer(i, j).color << 255, 255, 255, 255;
        }
    }
    auto triangleVertices = get_triangle_vertices(triangles);
    auto lineVertices = get_line_vertices(triangles);
    auto &uniform = viewer.uniform;
    int width = uniform.width;
    int height = uniform.height;
    // add insertion buffer to rendered buffer
    load_insertion_buffer_to_lines(viewer.insertionBuffer, lineVertices);

    color_black(lineVertices);
    rasterize_lines(program, uniform, lineVertices, 1, frameBuffer);
    rasterize_triangles(program, uniform, triangleVertices, frameBuffer);

    // Buffer for exchanging data between rasterizer and sdl viewer
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

    for (unsigned i = 0; i < frameBuffer.rows(); i++) {
        for (unsigned j = 0; j < frameBuffer.cols(); j++) {
            R(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(0);
            G(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(1);
            B(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(2);
            A(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(3);
        }
    }
    viewer.draw_image(R, G, B, A);
}

inline float linear_interpolate(float s, float e, float t) {
    return s+(e-s)*t;
}

std::function<void(SDLViewer &)> linear_animation(Program &program, FrameBuffer& frameBuffer) {
    return [&](SDLViewer &viewer) {
        if (!viewer.animate) {
            return;
        }
        viewer.animate = false;
        int middleFramesNum = 1000;
        std::vector<std::vector<Triangle>> frames((middleFramesNum) * (viewer.keyframes.size()-1)+1, std::vector<Triangle>());
        for (int i = 0; i < viewer.keyframes.size()-1; i++) {
            // adding frames
            auto &startFrame = viewer.keyframes[i];
            auto &endFrame = viewer.keyframes[i+1];
            for (int j = 0; j < startFrame.size(); j++) {
                // for each object in between
                // build interpolation function
                // they must be the same triangle, only coordinate are different
                auto &t1 = startFrame[j];
                auto &t2 = endFrame[j];
                for (int p = 0; p < middleFramesNum; p++) {
                    // interpolate
                    Triangle middle = t1;
                    float t = float(p)/float(middleFramesNum);
                    for (int k = 0; k < 3; k++) {
                        middle.vs[k].position.x() = linear_interpolate(t1.vs[k].position.x(), t2.vs[k].position.x(), t);
                        middle.vs[k].position.y() = linear_interpolate(t1.vs[k].position.y(), t2.vs[k].position.y(), t);
                    }
                    frames[i*middleFramesNum+p].push_back(middle);
                }
            }
        }
        for (auto &t: viewer.keyframes.back()) {
            frames.back().push_back(t);
        }

        for (auto &f: frames) {
            draw(program, frameBuffer, viewer, f);
        }

    };
}

Eigen::Vector4f bazier_recurse(const std::vector<Eigen::Vector4f> &current, float t) {
    if (current.size() == 1) {
        return current.front();
    }
    std::vector<Eigen::Vector4f> next;
    next.reserve(current.size()-1);
    for (int i = 0 ; i < current.size()-1; i++) {
        next.emplace_back(current[i]*(1-t) + current[i+1]*t);
    }
    return bazier_recurse(next, t);
}

std::function<void(SDLViewer &)> bezier_animation(Program &program, FrameBuffer& frameBuffer) {
    return [&](SDLViewer &viewer) {
        if (!viewer.animate) {
            return;
        }
        viewer.animate = false;
        int middleFramesNum = 500*viewer.keyframes.size();
        std::vector<std::vector<Triangle>> frames(middleFramesNum, std::vector<Triangle>());

        int objectNums = viewer.keyframes[0].size();
        for (int i = 0; i < objectNums; i++) {
            // current triangle
            auto &cur = viewer.keyframes[0][i];
            // get control points
            std::vector<Eigen::Vector4f> control_points;
            for (auto &kf: viewer.keyframes) {
                control_points.push_back(kf[i].centroid());
            }
            for (int j = 0; j < middleFramesNum; j++) {
                Triangle middle = cur;
                auto oldCenter = middle.centroid();
                auto newCenter = bazier_recurse(control_points, float(j)/float(middleFramesNum));
                auto offset = newCenter - oldCenter;
                for (auto &v: middle.vs) {
                    v.position += offset;
                }
                frames[j].push_back(middle);
            }
        }

        for (auto &f: frames) {
            draw(program, frameBuffer, viewer, f);
        }

    };
}

std::function<void(SDLViewer &)> default_redraw(Program &program, FrameBuffer& frameBuffer) {
    return [&](SDLViewer &viewer) {
        draw(program, frameBuffer, viewer, viewer.triangles);
    };
}

int main(int argc, char *args[]) {
    int width = 500;
    int height = 500;
    // The Framebuffer storing the image rendered by the rasterizer
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(width, height);

    // Global Constants (empty in this example)
//    UniformAttributes uniform;

    // Basic rasterization program
    Program program;

    // The vertex shader is the identity
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        // coordinate transformation
        VertexAttributes ret = va;

        ret.position = canvas_to_unit(ret.position, uniform.width, uniform.height);

        // apply view matrix
        ret.position = uniform.viewM * ret.position;
        ret.color = va.color;
        return ret;
    };

    // The fragment shader uses a fixed color
    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) -> FragmentAttributes {
        return {va.color(0), va.color(1), va.color(2)};
    };

    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    UniformAttributes &uniform = viewer.uniform;
    viewer.init("Viewer Example", width, height);

    uniform.width = width;
    uniform.height = height;

    viewer.mouse_move = [](int x, int y, int xrel, int yrel) {
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        if (key != 'n') {
            viewer.redraw = default_redraw(program, frameBuffer);
        }
        switch (key) {
            case 'i': {
                viewer.reset();
                viewer.mode = 'i';
                viewer.mouse_move = get_insertion_mouse_move(viewer);
                viewer.mouse_pressed = get_insertion_mouse_press(viewer);
                break;
            }
            case 'o': {
                viewer.reset();
                viewer.mode = 'o';
                viewer.mouse_pressed = get_translation_mouse_press(viewer);
                viewer.mouse_move = get_translation_mouse_move(viewer);
                break;
            }
            case 'p': {
                viewer.reset();
                viewer.mode = 'p';
                viewer.mouse_pressed = get_remove_mouse_press(viewer);
                break;
            }
            case 'h':
            case 'j':
            case 'k':
            case 'l':
                if (viewer.mode == 'o' && is_pressed) {
                    float degree = key == 'h' ? 10.f : (key == 'j' ? -10.f : 0.f);
                    float factor = key == 'k' ? 1.25f : (key == 'l' ? 0.75f : 1.f);
                    rotate_target_triangle(viewer, degree);
                    scale_target_triangle(viewer, factor);
                }
                viewer.redraw_next = true;
                break;
            case 'c':
                viewer.reset();
                viewer.mode = 'c';
                viewer.mouse_pressed = get_color_mouse_press(viewer);
                break;
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if (viewer.mode == 'c') {
                    color_vertex(viewer.colorTriIdx, viewer.colorVertexIdx, key, viewer);
                    viewer.redraw_next = true;
                }
                break;
            case '=':
                if (is_pressed) {
                    set_zoom_uniform_matrix(uniform, 1);
                    viewer.redraw_next = true;
                }
                break;
            case '-':
                if (is_pressed) {
                    set_zoom_uniform_matrix(uniform, -1);
                    viewer.redraw_next = true;
                }
                break;
            case 'w':
                if (is_pressed) {
                    set_translate_uniform_matrix(uniform, 0, 0.2f);
                    viewer.redraw_next = true;
                }
                break;
            case 'a':
                if (is_pressed) {
                    set_translate_uniform_matrix(uniform, -0.2f, 0);
                    viewer.redraw_next = true;
                }
                break;
            case 's':
                if (is_pressed) {
                    set_translate_uniform_matrix(uniform, 0, -0.2f);
                    viewer.redraw_next = true;
                }
                break;
            case 'd':
                if (is_pressed) {
                    set_translate_uniform_matrix(uniform, 0.2f, 0);
                    viewer.redraw_next = true;
                }
                break;
            case 'f':
                if (is_pressed) {
                    // create a keyframe, i.e. store current triangles
                    if (!viewer.triangles.empty()) {
                        viewer.keyframes.push_back(viewer.triangles);
                    }
                }
                break;
            case 'n':
                // play linear
                if (is_pressed) {
                    if (!viewer.keyframes.empty()) {
                        viewer.redraw = linear_animation(program, frameBuffer);
                        viewer.animate = true;
                    }
                    viewer.redraw_next = true;
                }
                break;
            case 'b':
                if (is_pressed) {
                    if (!viewer.keyframes.empty()) {
                        viewer.redraw = bezier_animation(program, frameBuffer);
                        viewer.animate = true;
                    }
                    viewer.redraw_next = true;
                }
                break;
            case 'x':
                if (is_pressed) {
                    viewer.keyframes.clear();
                }
            default:
                break;
        }
    };

    viewer.redraw = default_redraw(program, frameBuffer);

    viewer.launch();

    return 0;
}
