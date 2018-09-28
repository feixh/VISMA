//
// Created by feixh on 10/25/17.
//
#include "utils.h"

// system
#include "dirent.h"

// stl
#include <iostream>
#include <tuple>

// thirdparty
#include "igl/readOBJ.h"
#include "igl/readPLY.h"
#include "folly/json.h"
#include "folly/FileUtil.h"
#include "folly/Format.h"

// protocol buffer
#include "vlslam.pb.h"

namespace feh {

const std::string TermColor::red     = "\033[91m";
const std::string TermColor::green   = "\033[92m";
const std::string TermColor::blue    = "\033[94m";
const std::string TermColor::cyan    = "\033[96m";
const std::string TermColor::yellow  = "\033[93m";
const std::string TermColor::magenta = "\033[95m";
const std::string TermColor::gray    = "\033[90m";
const std::string TermColor::white   = "\033[97m";
const std::string TermColor::bold    = "\033[1m";
const std::string TermColor::end     = "\033[0m";
const std::string TermColor::endl     = "\033[0m\n";


std::ostream& operator<<(std::ostream& os, const Timer& obj)
{
    os << "....." << std::endl;
    // write obj to stream
    for ( auto it = obj.look_up_table_.begin(); it != obj.look_up_table_.end(); ++it ) {
        float elapsed = obj.report_average ? it->second / obj.counter_.at(it->first) : it->second;
        os << "[" << TermColor::bold+TermColor::green
           << obj.module_name_
           << TermColor::end << "]"
           << TermColor::cyan
           << it->first
           << TermColor::end
           << ":" << elapsed*1e-6 << " ms" << std::endl;
    }
    os << "....." << std::endl;
    return os;
}


bool Glob(const std::string &directory,
          const std::string &extension,
          std::vector<std::string> &filenames) {
    std::string suffix( (extension[0] == '.' ? "":".") + extension );
    std::string path(directory + "/");
    DIR *dir_ptr = opendir(path.c_str());
    struct dirent *entry;
    if (dir_ptr) {
        while ((entry = readdir(dir_ptr)) != NULL) {
            std::string entry_name(entry->d_name);
            if (entry_name.length() > suffix.length()
                && (entry_name.substr(entry_name.length() - suffix.length()).compare(suffix) == 0)) {
                filenames.push_back(entry_name.substr(0, entry_name.length() - suffix.length()));
            }
        }
        try {
            std::sort(filenames.begin(),
                      filenames.end(),
                      [](const std::string &a, const std::string &b) { return std::stof(a) < std::stof(b); });
        } catch (const std::invalid_argument &e) {
            std::sort(filenames.begin(),
                      filenames.end());
        }
        for (auto &filename: filenames) filename = path + filename + suffix;
        closedir(dir_ptr);
        return true;
    } else {
        return false;
    }
}

bool Glob(const std::string &directory,
          const std::string &extension,
          const std::string &prefix,
          std::vector<std::string> &filenames) {
    std::string suffix( (extension[0] == '.' ? "":".") + extension );
    std::string path(directory + "/");
    DIR *dir_ptr = opendir(path.c_str());
    struct dirent *entry;
    if (dir_ptr) {
        while ((entry = readdir(dir_ptr)) != NULL) {
            std::string entry_name(entry->d_name);
            if (entry_name.length() > suffix.length() + prefix.length()
                && (entry_name.substr(entry_name.length() - suffix.length()).compare(suffix) == 0)
                && (entry_name.substr(0, prefix.length()).compare(prefix) == 0)) {
                filenames.push_back(entry_name.substr(prefix.length(),
                                                      entry_name.length() - suffix.length() - prefix.length()));
            }
        }
        try {
            std::sort(filenames.begin(),
                      filenames.end(),
                      [](const std::string &a, const std::string &b) { return std::stof(a) < std::stof(b); });
        } catch (const std::invalid_argument &e) {
            std::sort(filenames.begin(),
                      filenames.end());
        }
        for (auto &filename: filenames) filename = path + prefix + filename + suffix;
        closedir(dir_ptr);
        return true;
    } else {
        return false;
    }
}

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
    bool success = igl::readPLY(ply_file, V, F);
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

