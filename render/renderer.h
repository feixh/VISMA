#pragma once

// stl
#include <string>
#include <iostream>
#include <memory>
#include <array>
#include <unordered_map>

#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "glog/logging.h"
#include "opencv2/core/core.hpp"

#include "shader.h"
#include "alias.h"

namespace feh {

////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS FOR THE RENDERER
////////////////////////////////////////////////////////////////////////////////
/// \brief: Check the id of current framebuffer under use
void CheckCurrentFramebufferId();
/// \brief: Print OpenGL and GLSL version information.
void PrintGLVersionInfo();
/// \brief: Convert buffered z value in depth buffer to actual depth.
/// \param zb: z in depth buffer
/// \param z_near: near plane distance
/// \param z_far: far plane distance
template <typename T>
T LinearizeDepth(T zb, T z_near, T z_far) {
    return 2 * z_near * z_far /
        (z_far + z_near - (2 * zb - 1) * (z_far - z_near));
}

////////////////////////////////////////////////////////////////////////////////
// THE RENDERER
////////////////////////////////////////////////////////////////////////////////
class Renderer {
public:
    Renderer(int maxHeight, int maxWidth); //, const std::string &name);
    ~Renderer();

    /// \brief: Set camera model.
    /// \param z_near: Near plane of camera.
    /// \param z_far: Far plane of camera.
    /// \param intrinsics: [fx, fy, cx, cy]
    void SetCamera(float z_near, float z_far, const float *intrinsics);
    void SetCamera(float z_near, float z_far, float fx, float fy, float cx, float cy);
    /// \brief: Set current camera pose.
    /// \param pose: transformation from **initial** camera frame to **current** camera frame.
    void SetCamera(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &pose);
    /// \brief: Set object mesh in canonical frame.
    /// \param vertices: Pointer to vertex array, flatten Nx3 array.
    /// \param num_vertices: Number of vertices.
    /// \param faces: Pointer to face array, flatten Mx3 array.
    /// \param num_faces: Number of faces.
    void SetMesh(float *vertices, int num_vertices, int *faces, int num_faces);
    void SetMesh(const std::vector<float> &vertices, const std::vector<int> &faces) {
        SetMesh(const_cast<float*>(&vertices[0]), vertices.size() / 3,
                const_cast<int*>(&faces[0]), faces.size()/3);
    }
    void SetMesh(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> vertices,
                 Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> faces) {
        SetMesh(vertices.data(), vertices.rows(), faces.data(), faces.rows());
    }

    /// \brief: Render the boundary of an object given object pose using Stencil Buffer.
    /// \param model: object pose
    /// \param out: pointer to output depth image
    void RenderDepth(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, float *out);
    void RenderDepth(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, cv::Mat &depth_map) {
        CHECK_EQ(depth_map.type(), CV_32FC1);
        CHECK_EQ(depth_map.rows, rows_);
        CHECK_EQ(depth_map.cols, cols_);
        RenderDepth(model, (float *)depth_map.data);
    }

    /// \brief: Render the edge of an object given object pose by:
    /// first render depth image and run edge detection on it.
    /// \param model: object pose
    /// \param out: pointer to output edge image
    void RenderEdge(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, uint8_t *out);
    void RenderEdge(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, cv::Mat &edge_map) {
        CHECK_EQ(edge_map.type(), CV_8UC1);
        CHECK_EQ(edge_map.rows, rows_);
        CHECK_EQ(edge_map.cols, cols_);
        RenderEdge(model, edge_map.data);
    }

    /// \brief: Render a binary mask given object pose.
    void RenderMask(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, uint8_t *out);
    void RenderMask(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model, cv::Mat &mask) {
        CHECK_EQ(mask.type(), CV_8UC1);
        CHECK_EQ(mask.rows, rows_);
        CHECK_EQ(mask.cols, cols_);
        RenderMask(model, mask.data);
    }


    void Use() { glfwMakeContextCurrent(window_); }


    // accessors
    float fx() const { return fx_; }
    float fy() const { return fy_; }
    float cx() const { return cx_; }
    float cy() const { return cy_; }
    float z_near() const { return z_near_; }
    float z_far() const { return z_far_; }
    int width() const { return cols_; }
    int cols() const { return cols_; }
    int height() const { return rows_; }
    int rows() const { return rows_; }
    const std::string &id() const { return name_; }
    const std::string &name() const { return name_; }

private:
    /// \brief Depth texture is mapped to the Quadrilateral (Quad) such that flipping/linearization/edge detection, etc.,
    /// and other effects can be applied to depth texture.
    void InitializeQuadrilateral();
    void InitializeFramebuffer();
    void InitializeForLikelihood();

private:
    static bool initialized_;
    static int counter_;
    bool output_with_GL_coordinate_system_;
    float fx_, fy_, cx_, cy_;
    float z_near_, z_far_;

private:
//private:
    int rows_, cols_;
    int num_vertices_, num_faces_;

    GLFWwindow *window_;
    std::string name_;
    GLuint fbo_; // framebuffer objcet
    GLuint vao_; // vertex attribute object
    GLuint vbo_; // vertex buffer object
    GLuint ebo_; // element/index buffer object

    GLuint vao_quad_, vbo_quad_, ebo_quad_; // For the quad

    GLuint color_texture_, depth_texture_; // color texture, depth texture

    std::vector<ShaderPtr> shader_list_;
    // We do NOT explicitly need a depth shader, since once depth test is enabled and there is a depth texture
    // bound to the framebuffer (the default comes with a depth texture, and for user-defined framebuffer, we need
    // to create and bind a depth texture to it), we can read the depth value from the depth texture for free.
    // Though we can make the fragment shader write out depth values as RGB image, this is essentially redudant.
    ShaderPtr depth_shader_;
    // Edge detection shader.
    ShaderPtr edge_shader_;
};

typedef std::shared_ptr<Renderer> RendererPtr;




}   // namespace feh


