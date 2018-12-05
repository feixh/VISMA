#include "renderer.h"

#include <chrono>
#include <GL/gl.h>

// GLM Mathematics
#ifndef GLM_FORCE_RADIANS
#define GLM_FORCE_RADIANS
#endif

#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/string_cast.hpp"
#include "glm/gtc/matrix_transform.hpp"

// For more advanced shaders, write them using GLSL and use the python script
// to generate constant string object and then include.
#include "shaders/basic_mvp_vert.i"
#include "shaders/position_texture_vert.i"
#include "shaders/edge_detection_frag.i"

namespace feh {

// constant color shaders
static const std::string black_frag_shader =
        R"(
#version 330 common
out vec4 color;
void main()
{
    color = vec4(0.0, 0.0, 0.0, 1.0);
}
)";

static const std::string white_frag_shader =
        R"(
#version 330 common
out vec4 color;
void main()
{
    color = vec4(1.0, 1.0, 1.0, 1.0);
}
)";

bool Renderer::initialized_ = false;
int Renderer::counter_ = 0;

Renderer::Renderer(int height, int width) : //, const std::string &name):
        output_with_GL_coordinate_system_(false),
        rows_(height),
        cols_(width),
        window_(nullptr),
        name_("GLRender" + std::to_string(counter_++)),
        color_texture_(0),
        depth_texture_(0),
        depth_shader_(nullptr),
        edge_shader_(nullptr)
{
    if (!initialized_) {
        glfwInit();
        initialized_ = true;
        LOG(INFO) << "glfw initialized";
        LOG(INFO) << "OpenGL version:" << glfwGetVersionString();
    }
    // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    // Create a GLFWwindow object that we can use for GLFW's functions
    window_ = glfwCreateWindow(cols_, rows_, name_.c_str(), nullptr, nullptr);
    glfwHideWindow(window_);
    glfwMakeContextCurrent(window_);
    LOG(INFO) << "window created";

    // Use glad (a loader generator) to initialize the OpenGL Function pointers
    if (!gladLoadGL()) {
        LOG(FATAL) << "FATAL::GLAD::failed to initialize OpnGL function pointers using glad";
    } else {
        LOG(INFO) << "OpenGL function pointers initialized with glad";
    }
    PrintGLVersionInfo();

    // Define the viewport dimensions
    glViewport(0, 0, cols_, rows_);

    depth_shader_ = std::make_shared<Shader>(basic_mvp_vert, "", "");

    edge_shader_ = std::make_shared<Shader>(position_texture_vert, edge_detection_frag, "");
    edge_shader_->SafeSetUniform("flip_y", not output_with_GL_coordinate_system_);
    edge_shader_->SafeSetUniform("z_near", 0.05f);
    edge_shader_->SafeSetUniform("z_far", 2.0f);

    LOG(INFO) << "shader(s) initialized";

    ///////////////////////////////////////////////////
    // Vertex Attribute Object
    ///////////////////////////////////////////////////
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    // Initialization Quad for texture manipulation
    InitializeQuadrilateral();
    LOG(INFO) << "Quadrilateral initialized";
    InitializeFramebuffer();
    LOG(INFO) << "Framebuffer initialized";
}

Renderer::~Renderer() {
    glfwMakeContextCurrent(window_);
    // clean up vertex buffers
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ebo_) glDeleteBuffers(1, &ebo_);

    // clean up quad buffers
    if (vao_quad_) glDeleteVertexArrays(1, &vao_quad_);
    if (vbo_quad_) glDeleteBuffers(1, &vbo_quad_);
    if (ebo_quad_) glDeleteBuffers(1, &ebo_quad_);

    if (color_texture_) glDeleteTextures(1, &color_texture_);
    if (depth_texture_) glDeleteTextures(1, &depth_texture_);
    if (fbo_) glDeleteFramebuffers(1, &fbo_);

    if (window_) glfwDestroyWindow(window_);

}

void Renderer::InitializeQuadrilateral() {
    // reference:
    // "Applying Texture" section of
    // https://learnopengl.com/#!Getting-started/Textures
    // And the source code:
    // https://learnopengl.com/code_viewer_gh.php?code=src/1.getting_started/4.1.textures/textures.cpp

    // To use depth texture as input of the edge detection algorithm, we need to create a quad abd attach the texture to
    // the quad (establish correspondences between quad vertices and texture coordinates). To achieve this, vertex attribute
    // arrays are used.
    float vert_quad[] = {
            // positions          // texture coords
            1.0f, 1.0f, 0.0f, 1.0f, 1.0f, // top right
            1.0f, -1.0f, 0.0f, 1.0f, 0.0f, // bottom right
            -1.0f, -1.0f, 0.0f, 0.0f, 0.0f, // bottom left
            -1.0f, 1.0f, 0.0f, 0.0f, 1.0f  // top left
    };
    int ind_quad[] = {
            0, 1, 3, // first triangle
            1, 2, 3  // second triangle
    };
    glGenVertexArrays(1, &vao_quad_);
    glGenBuffers(1, &vbo_quad_);
    glGenBuffers(1, &ebo_quad_);

    glBindVertexArray(vao_quad_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_quad_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vert_quad), vert_quad, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_quad_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ind_quad), ind_quad, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
#ifndef FEH_RENDER_USE_SEPARATE_ATTRIB_FORMAT
// position attribute
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *) 0);
// texture coord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *) (3 * sizeof(float)));
#else
    glBindVertexBuffer(0, vbo_quad_, 0, 5 * sizeof(float)); // 1st arg: binding index
    glVertexAttribFormat(0, 2, GL_FLOAT, GL_FALSE, 0);

    // in glBindVertexBuffer, 3rd argument is the offset of the first element w.r.t. starting point of the buffer.
    // in glVertexAttribFormat, last argument is the relative offset inside the data segment in the buffer.
    // Essentially, these two arguments control the same thing: we can either set the initial offset and make relative offset zero
    // OR we can make initial offset zero, and set the proper relative offset.
    glBindVertexBuffer(1, vbo_quad_, 3 * sizeof(float), 5 * sizeof(float));
    glVertexAttribFormat(1, 2, GL_FLOAT, GL_FALSE, 0);
    // 1st arg: attribute index; 2nd arg: binding index. attribute index and binding index are not required to be same.
    // attribute index is also useful when specifying layout of the attributes in shaders, while binding index not really
    // matter?
    glVertexAttribBinding(0, 0);
    glVertexAttribBinding(1, 1);
#endif


// Unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}


void Renderer::InitializeFramebuffer() {
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
// Create a color attachment texture
    glGenTextures(1, &color_texture_);
    glBindTexture(GL_TEXTURE_2D, color_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, cols_, rows_, 0, GL_RED, GL_FLOAT, NULL);
// setup texture buffer properties
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
// Attach the color texture to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture_, 0);

// Create a depth stencil texture
    glGenTextures(1, &depth_texture_);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, cols_, rows_, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, NULL);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, cols_, rows_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
// setup texture buffer properties
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);

//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//    glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
// attach depth & stencil texture to framebuffer
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, depth_stencil_buffer_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture_, 0);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        LOG(FATAL) << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!";
    glBindTexture(GL_TEXTURE_2D, 0);  // unbind texture
    glBindFramebuffer(GL_FRAMEBUFFER, 0); // unbind framebuffer
    LOG(INFO) << "Framebuffer initialized";
}

void Renderer::SetCamera(float z_near, float z_far, float fx, float fy, float cx, float cy) {
    glfwMakeContextCurrent(window_);
    float intrinsics[] = {fx, fy, cx, cy};
    SetCamera(z_near, z_far, intrinsics);
}

void Renderer::SetCamera(float zNear, float zFar, const float *intrinsics) {
    glfwMakeContextCurrent(window_);
    // store intrinsics
    fx_ = intrinsics[0];
    fy_ = intrinsics[1];
    cx_ = intrinsics[2];
    cy_ = intrinsics[3];
    z_near_ = zNear;
    z_far_ = zFar;

    // In OpenGL's view (camera) coordinate system, z is pointing toward us and y is pointing upward
    // In the conventional computer vision camera coordinate system, z is pointing forward and y is pointing to the floor.
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> vision_to_graphics;
    vision_to_graphics << 1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;

    // compute frustum & projection matrix from intrinsics
    float fcv[] = {intrinsics[0], intrinsics[1]};
    float ccv[] = {intrinsics[2], intrinsics[3]};
    float left = -ccv[0] / fcv[0] * zNear;
    float right = ((float) cols_ - 1.0 - ccv[0]) / fcv[0] * zNear;
//    float bottom = (ccv[1] - (float) (rows_ - 1)) / fcv[1] * zNear;
//    float top = ccv[1] / fcv[1] * zNear;

    // flip bottom and top
    // This is to cancel out the effects of applying vision_to_graphics transformation before.
    float bottom = ccv[1] / fcv[1] * zNear;
    float top = (ccv[1] - (float) (rows_ - 1)) / fcv[1] * zNear;
    glm::mat4 projection = glm::frustum(left, right, bottom, top, zNear, zFar);
//    projection = glm::scale(projection, glm::vec3(1, -1, 1));
    std::cout << "projection matrix=\n" << glm::to_string(projection) << "\n";


    if (depth_shader_) {
        depth_shader_->Use();
        glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "view"),
                           1, GL_FALSE,
                           vision_to_graphics.data());
        glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "projection"),
                           1, GL_FALSE,
                           glm::value_ptr(projection));
    }

}

void Renderer::SetCamera(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &pose) {
    glfwMakeContextCurrent(window_);
    // In OpenGL's view (camera) coordinate system, z is pointing toward us and y is pointing upward
    // In the conventional computer vision camera coordinate system, z is pointing forward and y is pointing to the floor.
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> vision_to_graphics;
    vision_to_graphics << 1, 0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1;
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> view = vision_to_graphics * pose;

    if (depth_shader_) {
        depth_shader_->Use();
        glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "view"), 1, GL_FALSE,
                           view.data());
    }
}


void Renderer::SetMesh(float *vertices, int num_vertices, int *faces, int num_faces) {
    glfwMakeContextCurrent(window_);
    num_vertices_ = num_vertices;
    num_faces_ = num_faces;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * num_vertices, vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * 3 * num_faces, faces, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid *) 0);
    glEnableVertexAttribArray(0);

    // unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Renderer::RenderDepth(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model_in, float *out) {
    glfwMakeContextCurrent(window_);
    // Render a depth map.
    glm::vec4 color(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_STENCIL_TEST);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    // Clear the color buffer & depth buffer
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Setup the shader for inner region of the object
    depth_shader_->Use();
    // Calculate the model matrix for each object and pass it to shader before drawing
    glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "model"), 1, GL_FALSE, model_in.data());
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, 3 * num_faces_, GL_UNSIGNED_INT, 0);

    if (out) {
        glReadPixels(0, 0, cols_, rows_, GL_DEPTH_COMPONENT, GL_FLOAT, out);
    }

    // unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    // unbind framebuffer ONLY AFTER copying values from graphic memory to normal memory
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer::RenderEdge(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model_in, uint8_t *out) {
//    model_ = glm::make_mat4(model_in.data());
    glfwMakeContextCurrent(window_);
    // Render a depth map.
    glm::vec4 color(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_STENCIL_TEST);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    // Bind to the color buffer
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    // Clear the color buffer & depth buffer
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Setup the shader for inner region of the object
    depth_shader_->Use();
    // Calculate the model matrix for each object and pass it to shader before drawing
    glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "model"), 1, GL_FALSE, model_in.data());
    // Bind Vertex Array Object
    glBindVertexArray(vao_);
    // Draw
    glDrawElements(GL_TRIANGLES, 3 * num_faces_, GL_UNSIGNED_INT, 0);

    // apply the flip shader
    glDisable(GL_DEPTH_TEST);   // disable depth test since we are drawing anyway ...
    // apply edge detection shader
    edge_shader_->Use();
    // bind texture
    edge_shader_->SafeSetUniform("this_texture", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, depth_texture_);

    glBindVertexArray(vao_quad_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    if (out) {
        // No need to manually flip values, since the shader handles this.
        glReadPixels(0, 0, cols_, rows_, GL_RED, GL_UNSIGNED_BYTE, out);
    }

    // unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    // unbind framebuffer ONLY AFTER copying values from graphic memory to normal memory
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void Renderer::RenderMask(const Eigen::Matrix<float, 4, 4, Eigen::ColMajor> &model_in, uint8_t *out) {
    glfwMakeContextCurrent(window_);
    // Render a depth map.
    glm::vec4 color(1.0, 1.0, 1.0, 1.0);
    glDisable(GL_STENCIL_TEST);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    // Clear the color buffer & depth buffer
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    // Setup the shader for inner region of the object
    depth_shader_->Use();
    // Calculate the model matrix for each object and pass it to shader before drawing
    glUniformMatrix4fv(glGetUniformLocation(depth_shader_->Program, "model"), 1, GL_FALSE, model_in.data());
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, 3 * num_faces_, GL_UNSIGNED_INT, 0);

    if (out) {
        glReadPixels(0, 0, cols_, rows_, GL_RED, GL_UNSIGNED_BYTE, out);
    }

    // unbind texture
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindVertexArray(0);
    // unbind framebuffer ONLY AFTER copying values from graphic memory to normal memory
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS FOR THE RENDERER
////////////////////////////////////////////////////////////////////////////////
void PrintGLVersionInfo() {
    const GLubyte *renderer = glGetString(GL_RENDERER);
    const GLubyte *vendor = glGetString(GL_VENDOR);
    const GLubyte *version = glGetString(GL_VERSION);

    const GLubyte *glsl_version = glGetString(GL_SHADING_LANGUAGE_VERSION);
    LOG(INFO) << "GL Renderer" << vendor;
    LOG(INFO) << "GL Verndor" << renderer;
    LOG(INFO) << "GL Version" << version;
    LOG(INFO) << "GLSL Version" << glsl_version;
}

void CheckCurrentFramebufferId() {
    GLint drawFboId = 0, readFboId = 0;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &drawFboId);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readFboId);
    LOG(INFO) << "before: draw fb id=" << drawFboId << " " << "read fb id=" << readFboId;
}


}   // namespace feh


