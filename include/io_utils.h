#pragma once
#include "common/eigen_alias.h"

// stl
#include <string>
#include <vector>
#include <fstream>

// 3rd party
#include "opencv2/core.hpp"
#include "folly/dynamic.h"
#include "glog/logging.h"

#include "common/utils.h"

namespace feh {

namespace io {

class MeshIO : public std::exception {
    virtual const char* what() const throw() {
        return "Mesh IO Exception";
    }
};
/// \brief: Load vertices and faces from an .obj file.
/// \param obj_file: The .obj file.
/// \param vertices: Vertices of the mesh.
/// \param faces: Faces of the mesh.
bool LoadMeshFromObjFile(const std::string &obj_file, std::vector<float> &vertices, std::vector<int> &faces);
std::tuple<std::vector<float>, std::vector<int>> LoadMeshFromObjFile(const std::string &obj_file);

bool LoadMeshFromPlyFile(const std::string &obj_file, std::vector<float> &vertices, std::vector<int> &faces);
std::tuple<std::vector<float>, std::vector<int>> LoadMeshFromPlyFile(const std::string &obj_file);

/// \brief: Load edgemap from protobuf file.
bool LoadEdgeMap(const std::string &filename, cv::Mat &edge);
/// \brief: Load a list of mesh file paths.
/// \param root: Root directory of the CAD database. All the meshes are put directly under this directory.
/// \param cat_json: Json file of the list of meshes of a certain category.
std::vector<std::string> LoadMeshDatabase(const std::string &root, const std::string &cat_json);

/// \brief: Convert protobuf repeated field to Eigen matrix.
Mat4f SE3FromArray(float *data);
Mat4f SE3FromArray(double *data);

/// \brief load NxM double matrix from json file
template<typename T=FloatType, int N=3, int M = N>
Eigen::Matrix<T, N, M> GetMatrixFromDynamic(
    const folly::dynamic &v,
    const std::string &key) {

    Eigen::Matrix<T, N, M> ret;
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j)
            ret(i, j) = v[key][i * M + j].asDouble();
    return ret;
}

/// \brief load N-dim double vector from json file
template<typename T=FloatType, int N>
Eigen::Matrix<T, N, 1> GetVectorFromDynamic(
    const folly::dynamic &v,
    const std::string &key) {

    return GetMatrixFromDynamic<T, N, 1>(v, key);
};

template<typename Derived>
void WriteMatrixToDynamic(folly::dynamic &d, const std::string &key, const Eigen::MatrixBase<Derived> &m) {
    d[key] = folly::dynamic::array();
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < m.cols(); ++j)
            d[key].push_back(m(i, j));
}

/// \brief: Save opencv mat to file in txt or binary form (if the flag binary is turned on).
template <typename T>
void SaveMatToFile(const std::string &filename, const cv::Mat &mat, bool binary=false) {
    CHECK(!binary) << "binary mode not implemented";
    std::ofstream ofs(filename, std::ios::out);
    CHECK(ofs.is_open()) << "failed to open output file @ " << filename;
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            ofs << (float) mat.at<T>(i, j) << " ";
        }
        ofs << "\n";
    }
    ofs.close();
}

/// \brief: Merge two dynamic objects and return the merged one.
folly::dynamic MergeDynamic(const folly::dynamic &a, const folly::dynamic &b);


} // namespace io

}   // namespace feh


