#include "io_utils.h"

// stl
#include <tuple>

// thirdparty
#include "igl/readOBJ.h"
#include "igl/readPLY.h"
#include "folly/json.h"
#include "folly/FileUtil.h"
#include "folly/Format.h"

// own
#include "vlslam.pb.h"

namespace feh {

namespace io {

bool LoadMeshFromObjFile(const std::string &obj_file,
                         std::vector<float> &vertices,
                         std::vector<int> &faces) {
    Eigen::MatrixXf V;
    Eigen::MatrixXi F;
    bool success = igl::readOBJ(obj_file, V, F);
    if (success) {
        vertices.reserve(V.rows() * 3);
        for (int i = 0; i < V.rows(); ++i) {
            vertices.insert(vertices.end(), {V(i, 0), V(i, 1), V(i, 2)});
        }
        faces.reserve(F.rows() * 3);
        for (int i = 0; i < F.rows(); ++i) {
            faces.insert(faces.end(), {F(i, 0), F(i, 1), F(i, 2)});
        }
    }
    return success;
}

std::tuple<std::vector<float>, std::vector<int>> LoadMeshFromObjFile(const std::string &obj_file) {
    std::vector<float> vertices;
    std::vector<int> faces;
    if (LoadMeshFromObjFile(obj_file, vertices, faces)) {
        return std::make_tuple(vertices, faces);
    } else throw MeshIO();
};

bool LoadMeshFromPlyFile(const std::string &ply_file,
                         std::vector<float> &vertices,
                         std::vector<int> &faces) {
    Eigen::MatrixXf V;
    Eigen::MatrixXi F;
    igl::readPLY(ply_file, V, F);
    vertices.reserve(V.rows() * 3);
    for (int i = 0; i < V.rows(); ++i) {
        vertices.insert(vertices.end(), {V(i, 0), V(i, 1), V(i, 2)});
    }
    faces.reserve(F.rows() * 3);
    for (int i = 0; i < F.rows(); ++i) {
        faces.insert(faces.end(), {F(i, 0), F(i, 1), F(i, 2)});
    }
}

std::tuple<std::vector<float>, std::vector<int>> LoadMeshFromPlyFile(const std::string &ply_file) {
    std::vector<float> vertices;
    std::vector<int> faces;
    if (LoadMeshFromPlyFile(ply_file, vertices, faces)) {
        return std::make_tuple(vertices, faces);
    } else throw MeshIO();
};



bool LoadEdgeMap(const std::string &filename, cv::Mat &edge) {
    vlslam_pb::EdgeMap edgemap;
    try {
        std::ifstream in_file(filename);
        edgemap.ParseFromIstream(&in_file);
        in_file.close();
        edge = cv::Mat(edgemap.rows(), edgemap.cols(),
                       CV_32FC1,
                       edgemap.mutable_data()->mutable_data());
        edge.convertTo(edge, CV_8UC1, 255.0f);
        return true;
    } catch (const std::exception &e) {
        return false;
    }
}


std::vector<std::string> LoadMeshDatabase(const std::string &root, const std::string &cat_json) {
    CHECK_STREQ(cat_json.substr(cat_json.find('.'), 5).c_str(), ".json");
    std::string content;
    std::string full_path = folly::sformat("{}/{}", root, cat_json);
    folly::readFile(full_path.c_str(), content);
    folly::dynamic json_content = folly::parseJson(folly::json::stripComments(content));
    std::vector<std::string> out;
    for (const auto &value : json_content["entries"]) {
        out.push_back(value.asString());
    }
    return out;
}

Mat4f SE3FromArray(float *data) {
    Mat4f out;
    out.block<3, 4>(0, 0) = Eigen::Map<Eigen::Matrix<float, 3, 4, Eigen::RowMajor>>(data);
    out(3, 3) = 1.0f;
    return out;
}

Mat4f SE3FromArray(double *data) {
    Mat4d out;
    out.block<3, 4>(0, 0) = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(data);
    out(3, 3) = 1.0f;
    return out.cast<float>();
}

folly::dynamic MergeDynamic(const folly::dynamic &a, const folly::dynamic &b) {
    if (b == nullptr) return a;
    folly::dynamic out(a);
    for (auto &item : b.items()) {
        if (out.find(item.first) != out.items().end()) {
            std::cout << TermColor::yellow << "WARNING: KEY " << item.first << " EXISTS" << TermColor::endl;
        }
        out[item.first] = item.second;
    }
    return out;
}

}   // namespace io

}   // namespace feh
