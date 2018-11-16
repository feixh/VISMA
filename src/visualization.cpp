//
// Created by visionlab on 3/2/18.
//
#include "tool.h"

#include "json/json.h"
// libigl
#include "igl/readOBJ.h"
#include "igl/writeOBJ.h"
#include "igl/readPLY.h"
// Open3D
#include "IO/IO.h"
#include "Visualization/Visualization.h"
// OpenCV
#include "opencv2/opencv.hpp"

// feh
#include "utils.h"
#include "geometry.h"
#include "dataloader.h"
#include "vlslam.pb.h"

namespace feh {

void AssembleScene(const Json::Value &config,
                   const std::list<std::pair<std::string, Eigen::Matrix<double, 3, 4>>> &objects,
                   const Eigen::Matrix<double, 3, 4> &alignment,
                   std::vector<Eigen::Matrix<double, 3, 1>> &vertices,
                   std::vector<Eigen::Matrix<int, 3, 1>> &faces) {
    std::string database_dir = config["CAD_database_root"].asString();
    std::string dataroot = config["dataroot"].asString();
    std::string dataset = config["dataset"].asString();
    std::string scene_dir = dataroot + "/" + dataset + "/";
    std::string fragment_dir = scene_dir + "/fragments/";

    // LOAD FLAGS
    bool show_original = config["scene_assembler"].get("show_original_scene", true).asBool();
    bool remove_original = config["scene_assembler"].get("remove_original_objects", true).asBool();
    double padding_size = config["scene_assembler"].get("padding_size", 0.0).asDouble();

    // LOAD SCENE POINT CLOUD
    std::list<Eigen::Matrix<double, 3, 1>> sceneV;
    if (show_original) {
        auto scene = std::make_shared<open3d::PointCloud>();
        try {
            open3d::ReadPointCloudFromPLY(scene_dir + "/test.klg.ply", *scene);
            for (int i = 0; i < scene->points_.size(); ++i) {
                sceneV.push_back({});
                sceneV.back().head<3>() = scene->points_[i];
                // sceneV.back().tail<3>() = scene->colors_[i]; // / 255.0;
            }
        } catch (...) {
            std::cout << TermColor::bold + TermColor::red << "GROUND TRUTH POINT CLOUD NOT EXIST" << TermColor::endl;
        }
    }

    for (const auto &obj : objects) {
        std::string model_name = obj.first;
        auto pose = obj.second;
        // LOAD MESH
        Eigen::Matrix<double, Eigen::Dynamic, 3> v;
        Eigen::Matrix<int, Eigen::Dynamic, 3> f;
        Eigen::Matrix<double, Eigen::Dynamic, 6> tmp;
        igl::readOBJ(absl::StrFormat("%s/%s.obj", database_dir, model_name), tmp, f);
        v = tmp.leftCols(3);
        std::cout << "v.size=" << v.rows() << "x" << v.cols() << "\n";
        // TRANSFORM TO SCENE FRAME
        v.leftCols(3) = (v.leftCols(3) * pose.block<3,3>(0,0).transpose()).rowwise() + pose.block<3, 1>(0, 3).transpose();
        v.leftCols(3) = (v.leftCols(3) * alignment.block<3,3>(0,0).transpose()).rowwise() + alignment.block<3, 1>(0, 3).transpose();
        int v_offset = vertices.size();
        for (int i = 0; i < v.rows(); ++i) {
            vertices.push_back(v.row(i).head<3>());
        }
        if (show_original && !sceneV.empty() && remove_original) {
            std::array<Eigen::Vector3d, 2> bounds;
            bounds[0] = Eigen::Vector3d::Ones() * std::numeric_limits<double>::max();
            bounds[1] = Eigen::Vector3d::Ones() * std::numeric_limits<double>::lowest();
            for (int i = 0; i < v.rows(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    if (v(i, j) < bounds[0](j)) bounds[0](j) = v(i, j);
                    if (v(i, j) > bounds[1](j)) bounds[1](j) = v(i, j);
                }
            }
            // expand the bounds a little bit
            bounds[0].array() -= padding_size;
            bounds[1].array() += padding_size;
            for (auto it = sceneV.begin(); it != sceneV.end(); ) {
                Eigen::Vector3d v = it->head<3>();
                bool in_bound = true;
                for (int i = 0; i < 3; ++i)
                    if (!(v(i) > bounds[0](i) && v(i) < bounds[1](i))) {
                        in_bound = false;
                        break;
                    }
                if (in_bound) {
                    it = sceneV.erase(it);
                } else ++it;
            }
        }

        for (int i = 0; i < f.rows(); ++i) {
            faces.push_back({v_offset + f(i, 0), v_offset + f(i, 1), v_offset + f(i, 2)});
        }
    }
    if (!sceneV.empty()) vertices.insert(vertices.end(), sceneV.begin(), sceneV.end());
}

void AssembleResult(const Json::Value &config,
                    Eigen::Matrix<double, Eigen::Dynamic, 3> *Vout,
                    Eigen::Matrix<int, Eigen::Dynamic, 3> *Fout,
                    std::vector<Eigen::Matrix<double, 3, 4>> *Gout) {
    // EXTRACT PATHS
    std::string database_dir = config["CAD_database_root"].asString();

    std::string dataroot = config["dataroot"].asString();
    std::string dataset = config["dataset"].asString();
    std::string scene_dir = dataroot + "/" + dataset + "/";
    std::string fragment_dir = scene_dir + "/fragments/";

    // FILE I/O BUFFER
    std::string contents;

    // READ THE CORVIS TO ELASTICFUSION ALIGNMENT
    std::string result_alignment_file = scene_dir + "/result_alignment.json";
    Eigen::Matrix<double, 3, 4> T_ef_corvis;
    try {
        auto result_alignment = LoadJson(result_alignment_file);
        T_ef_corvis = GetMatrixFromJson<double, 3, 4>(result_alignment, "T_ef_corvis").block<3, 4>(0, 0);
    } catch (...) {
        std::cout << TermColor::bold + TermColor::red << "failed to load result alignment; use identity transformation!!!" << TermColor::endl;
        T_ef_corvis.block<3, 3>(0, 0).setIdentity();
    }
    std::cout << "result_alignment=\n" << T_ef_corvis << "\n";


    // LOAD RESULT FILE
    std::string result_file = scene_dir + "/result.json";
    auto result = LoadJson(result_file);
    // ITERATE AND GET THE LAST ONE
    int result_index = config["result_visualization"].get("result_index", -1).asInt();
    if (result_index < 0) result_index = result.size()-1;
    auto packet = result[result_index];
    std::list<std::pair<std::string, Eigen::Matrix<double, 3, 4>>> objects;
    for (const auto &obj : packet) {
        auto pose = GetMatrixFromJson<double, 3, 4>(obj, "model_pose");
        std::cout << absl::StrFormat("id=%d\nstatus=%d\nshape=%s\npose=\n",
                                   obj["id"].asInt(),
                                   obj["status"].asInt(),
                                   obj["model_name"].asString())
                  << pose << "\n";
        std::string model_name = obj["model_name"].asString();
        objects.push_back(std::make_pair(model_name, pose));
    }

    std::vector<Eigen::Matrix<double, 3, 1>> vertices;
    std::vector<Eigen::Matrix<int, 3, 1>> faces;
    AssembleScene(config, objects, T_ef_corvis, vertices, faces);
    igl::writeOBJ(scene_dir + "/result_augmented_view.obj",
                  StdVectorOfEigenVectorToEigenMatrix(vertices),
                  StdVectorOfEigenVectorToEigenMatrix(faces));

    if (Vout && Fout) {
        *Vout = StdVectorOfEigenVectorToEigenMatrix(vertices);
        *Fout = StdVectorOfEigenVectorToEigenMatrix(faces);
    }

    if (Gout) {
        for (const auto &obj : objects) {
            auto R = T_ef_corvis.block<3, 3>(0, 0) * obj.second.block<3, 3>(0, 0);
            auto T = T_ef_corvis.block<3, 3>(0, 0) * obj.second.block<3, 1>(0, 3) + T_ef_corvis.block<3, 1>(0, 3);
            Gout->push_back((Eigen::Matrix<double, 3, 4>() << R, T).finished());
        }
    }
}

void AssembleGroundTruth(const Json::Value &config,
                         Eigen::Matrix<double, Eigen::Dynamic, 3> *Vout,
                         Eigen::Matrix<int, Eigen::Dynamic, 3> *Fout,
                         std::vector<Eigen::Matrix<double, 3, 4>> *Gout) {
    std::string database_dir = config["CAD_database_root"].asString();
    std::string dataroot = config["dataroot"].asString();
    std::string dataset = config["dataset"].asString();
    std::string scene_dir = dataroot + "/" + dataset + "/";
    std::string fragment_dir = scene_dir + "/fragments/";

    // LOAD GROUND TRUTH ALIGNMENT
    std::string alignment_file = fragment_dir + "/alignment.json";
    auto alignment = LoadJson(alignment_file);

    std::list<std::pair<std::string, Eigen::Matrix<double, 3, 4>>> objects;
    // for (const auto &obj : alignment.keys()) {
    for (auto it = alignment.begin(); it != alignment.end(); ++it) {
        std::string obj_name = it.key().asString();
        std::string model_name = obj_name.substr(0, obj_name.find_last_of('_'));
        auto pose = GetMatrixFromJson<double, 3, 4>(alignment, obj_name);
        std::cout << obj_name << "\n" << model_name << "\n" << pose << "\n";
        objects.push_back(std::make_pair(model_name, pose));
    }
    Eigen::Matrix<double, 3, 4> identity;
    identity.setZero();
    identity(0, 0) = 1; identity(1, 1) = 1; identity(2, 2) = 1;

    std::vector<Eigen::Matrix<double, 3, 1>> vertices;
    std::vector<Eigen::Matrix<int, 3, 1>> faces;

    AssembleScene(config, objects, identity, vertices, faces);
    igl::writeOBJ(scene_dir + "/ground_truth_augmented_view.obj",
                  StdVectorOfEigenVectorToEigenMatrix(vertices),
                  StdVectorOfEigenVectorToEigenMatrix(faces));

    if (Vout && Fout) {
        *Vout = StdVectorOfEigenVectorToEigenMatrix(vertices);
        *Fout = StdVectorOfEigenVectorToEigenMatrix(faces);
    }

    if (Gout) {
        for (const auto &obj : objects) {
            auto R = identity.block<3, 3>(0, 0) * obj.second.block<3, 3>(0, 0);
            auto T = identity.block<3, 3>(0, 0) * obj.second.block<3, 1>(0, 3)
                + identity.block<3, 1>(0, 3);
            Gout->push_back((Eigen::Matrix<double, 3, 4>() << R, T).finished());
        }
    }
}

void VisualizeResult(const Json::Value &config) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> V, Vtot;
    Eigen::Matrix<int, Eigen::Dynamic, 3> F;
    AssembleResult(config, &V, &F);

    std::string dataset_path = config["experiment_root"].asString() + "/" + config["dataset"].asString();
    std::string scene_dir = absl::StrFormat("%s/%s/", 
        config["dataroot"].asString(), config["dataset"].asString());

    Eigen::Matrix<double, Eigen::Dynamic, 6> traj, pts;

    if (config["result_visualization"]["show_trajectory"].asBool()) {
        std::string contents;
        std::string result_alignment_file = scene_dir + "/result_alignment.json";
        Eigen::Matrix<double, 3, 4> T_ef_corvis;
        try {
            auto result_alignment = LoadJson(result_alignment_file);
            T_ef_corvis = GetMatrixFromJson<double, 3, 4>(result_alignment, "T_ef_corvis").block<3, 4>(0, 0);
        } catch (...) {
            std::cout << TermColor::bold + TermColor::red << "failed to load result alignment; use identity transformation!!!" << TermColor::endl;
            T_ef_corvis.block<3, 3>(0, 0).setIdentity();
        }
        std::cout << "result_alignment=\n" << T_ef_corvis << "\n";


        std::ifstream in_file(dataset_path + "/dataset");
        CHECK(in_file.is_open()) << "failed to open dataset @ " << dataset_path;
        vlslam_pb::Dataset dataset;
        dataset.ParseFromIstream(&in_file);
        in_file.close();
        std::vector<Eigen::Matrix<double, 6, 1>> vtraj;
        for (int i = 0; i < dataset.packets_size(); ++i) {
            auto packet = dataset.mutable_packets(i);
            const auto gwc = SE3f(SE3FromArray(packet->mutable_gwc()->mutable_data()));
            auto Twc = T_ef_corvis.block<3, 3>(0, 0) * gwc.translation().cast<double>() + T_ef_corvis.block<3, 1>(0, 3);
            vtraj.push_back({});
            vtraj.back() << Twc(0), Twc(1), Twc(2), 1.0, 0.5, 0.0;
        }
        traj = StdVectorOfEigenVectorToEigenMatrix(vtraj);
    }

    if (config["result_visualization"]["show_pointcloud"].asBool()) {
        std::string contents;
        std::string result_alignment_file = scene_dir + "/result_alignment.json";
        Eigen::Matrix<double, 3, 4> T_ef_corvis;
        try {
            auto result_alignment = LoadJson(result_alignment_file);
            T_ef_corvis = GetMatrixFromJson<double, 3, 4>(result_alignment, "T_ef_corvis").block<3, 4>(0, 0);
        } catch (...) {
            std::cout << TermColor::bold + TermColor::red << "failed to load result alignment; use identity transformation!!!" << TermColor::endl;
            T_ef_corvis.block<3, 3>(0, 0).setIdentity();
        }
        std::cout << "result_alignment=\n" << T_ef_corvis << "\n";


        std::ifstream in_file(dataset_path + "/dataset");
        CHECK(in_file.is_open()) << "failed to open dataset @ " << dataset_path;
        vlslam_pb::Dataset dataset;
        dataset.ParseFromIstream(&in_file);
        in_file.close();
        std::unordered_map<uint64_t, Eigen::Matrix<double, 6, 1>> mpts;
        for (int i = 0; i < dataset.packets_size(); ++i) {
            auto packet = dataset.packets(i);
            for (int j = 0; j < packet.features_size(); ++j) {
                auto feature = packet.features(j);
                if (feature.status() == vlslam_pb::Feature::INSTATE) {
                    mpts[feature.id()] << T_ef_corvis.block<3, 3>(0, 0)
                        * Eigen::Vector3d{feature.xw(0), feature.xw(1), feature.xw(2)}
                        + T_ef_corvis.block<3, 1>(0, 3),
                        0, 0.5, 1.0;
                }
            }
        }
        // convert map to eigen matrix
        pts.resize(mpts.size(), 6);
        int counter(0);
        for (auto it = mpts.begin(); it != mpts.end(); ++it) {
            pts.row(counter++) = it->second;
        }
    }

    // ASSEMBLE
    Vtot.resize(V.rows() + traj.rows() + pts.rows(), 6);
    Vtot << V, traj, pts;
    igl::writeOBJ(scene_dir + "/result_with_trajectory_and_pointcloud.obj",
                  Vtot, F);

}


}
