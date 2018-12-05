#pragma once

#include <memory>
#include <type_traits>

namespace feh {

class Shader {
public:
    GLuint Program;
    std::string name_;

    // Constructor generates the shader on the fly
    Shader(const std::string &vertex_code, const std::string &fragment_code, const std::string &compute_code) {
        name_ = vertex_code + fragment_code + compute_code;
        const GLchar *vertex_shader_code = vertex_code.c_str();
        const GLchar *fragment_shader_code = fragment_code.c_str();
        const GLchar *compute_shader_code = compute_code.c_str();
        bool has_vs(true), has_fs(true), has_cs(true);
        if (vertex_code.empty()) {
            LOG(INFO) << "Empty Vertex Shader";
            has_vs = false;
        }
        if (fragment_code.empty()) {
            LOG(INFO) << "Empty Fragment Shader";
            has_fs = false;
        }
        if (compute_code.empty()) {
            LOG(INFO) << "Empty Compute Shader";
            has_cs = false;
        }
        if (!(has_vs || has_fs || has_cs)) LOG(FATAL) << "NO shader has been provided";

        // Shader Program
        this->Program = glCreateProgram();

        // 2. Compile shaders
        GLuint vertex(0), fragment(0), compute(0);
        GLint success;
        GLchar infoLog[512];

        // Vertex Shader
        if (has_vs) {
            vertex = glCreateShader(GL_VERTEX_SHADER);
            glShaderSource(vertex, 1, &vertex_shader_code, NULL);
            glCompileShader(vertex);
            // Print compile errors if any
            glGetShaderiv(vertex, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(vertex, 512, NULL, infoLog);
                LOG(FATAL) << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog;
            }
            glAttachShader(this->Program, vertex);
        }

        if (has_fs) {
            // Fragment Shader
            fragment = glCreateShader(GL_FRAGMENT_SHADER);
            glShaderSource(fragment, 1, &fragment_shader_code, NULL);
            glCompileShader(fragment);
            // Print compile errors if any
            glGetShaderiv(fragment, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(fragment, 512, NULL, infoLog);
                LOG(FATAL) << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog;
            }
            glAttachShader(this->Program, fragment);
        }

        if (has_cs) {
            // Fragment Shader
            compute = glCreateShader(GL_COMPUTE_SHADER);
            glShaderSource(compute, 1, &compute_shader_code, NULL);
            glCompileShader(compute);
            // Print compile errors if any
            glGetShaderiv(compute, GL_COMPILE_STATUS, &success);
            if (!success) {
                glGetShaderInfoLog(compute, 512, NULL, infoLog);
                LOG(FATAL) << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog;
            }
            glAttachShader(this->Program, compute);
        }


        // Link
        glLinkProgram(this->Program);
        glGetProgramiv(this->Program, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(this->Program, 512, NULL, infoLog);
            LOG(FATAL) << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog;
        }

        // Delete the shaders as they're linked into our program now and no longer needed
        if (has_vs) glDeleteShader(vertex);
        if (has_fs) glDeleteShader(fragment);
        if (has_cs) glDeleteShader(compute);

    }

    // Uses the current shader
    void Use() {
        glUseProgram(this->Program);
    }

    /// \brief: Set uniform variable.
    /// \param uniform_name: The uniform variable.
    /// \param val: Value to set.
    template <typename T>
    bool SetUniform(const std::string &uniform_name, T val) {
        glUseProgram(this->Program);
        GLuint loc = glGetUniformLocation(this->Program, uniform_name.c_str());
        if (loc == -1) {
            LOG(WARNING) << "inactive uniform: " << uniform_name << " ? check spell";
            return true;
        }

        if (std::is_same<T, int>::value || std::is_same<T, bool>::value) {
//            std::cout << uniform_name << " is int\n";
            glUniform1i(loc, val);
        } else if (std::is_same<T, float>::value || std::is_same<T, double>::value) {
//            std::cout << uniform_name << " is float\n";
            glUniform1f(loc, val);
        } else {
            LOG(ERROR) << "invalid value type; can only handle float and int for now";
            return false;
        }
        return true;
    }

    template <typename T>
    void SafeSetUniform(const std::string &uniform_name, T val) {
        CHECK(SetUniform(uniform_name, val))
        << "failed to set " << uniform_name << " for " << name_;
    }

    template <typename T>
    bool SetUniform(const std::string &uniform_name, int size, T *address) {
        glUseProgram(this->Program);
        GLuint loc = glGetUniformLocation(this->Program, uniform_name.c_str());
        if (loc == -1) {
            LOG(WARNING) << "inactive uniform: " << uniform_name << " ? check spell";
            return true;
        }

        if (std::is_same<T, int>::value) {
//            std::cout << uniform_name << " is int\n";
            // FIXME: weird bug here
            glUniform1iv(loc, size, address);
        } else if (std::is_same<T, float>::value) {
//            std::cout << uniform_name << " is float\n";
            glUniform1fv(loc, size, address);
        } else {
            LOG(ERROR) << "invalid value type; can only handle float and int for now";
            return false;
        }
        return true;
    }

    template <typename T>
    void SafeSetUniform(const std::string &uniform_name, int size, T *address) {
        CHECK(SetUniform(uniform_name, size, address))
        << "failed to set " << uniform_name << " for " << name_;
    }

};

typedef std::shared_ptr<Shader> ShaderPtr;


}   // namespace feh
