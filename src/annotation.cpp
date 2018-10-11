//
// Created by visionlab on 2/20/18.
//
#include "tool.h"

// 3rd party
#include "IO/IO.h"
#include "Visualization/Visualization.h"
#include "Core/Core.h"
#include "igl/readOBJ.h"
#include "sophus/se3.hpp"
#include "folly/FileUtil.h"
#include "folly/Format.h"
#include "folly/json.h"

// feh
#include "constrained_icp.h"
#include "geometry.h"
#include "io_utils.h"

namespace feh {

double MinY(const std::vector<Eigen::Vector3d> &v) {
    double min_y = std::numeric_limits<double>::max();
    for (const auto &vec : v) {
        if (vec[1] < min_y) min_y = vec[1];
    }
    return min_y;
}

Eigen::Matrix4d RegisterModelToScene(std::shared_ptr <three::PointCloud> model,
                                     std::shared_ptr <three::PointCloud> scene,
                                     const folly::dynamic options) {
    int level = options["rotation_level"].getInt();
    double threshold = options["distance_threshold"].asDouble();

    double interval = 2 * M_PI / level;
    three::RegistrationResult best_result;
    for (int i = 0; i < level; ++i) {
        std::cout << "try " << 360 / level * i << " degree\n";
        Eigen::Matrix4d init_trans;
        init_trans.setIdentity();
        init_trans.block<3, 3>(0, 0) = Eigen::AngleAxis < double > {
            interval * i,
            Eigen::Vector3d::UnitY()}.toRotationMatrix();
        three::RegistrationResult result;
        if (options["point_to_plane"].asBool()) {
            result = three::RegistrationICP(
                *model, *scene,
                threshold, init_trans,
                three::TransformationEstimationPointToPlane(),
                three::ICPConvergenceCriteria());
        } else {
            result = three::RegistrationICP(
                *model, *scene,
                threshold, init_trans,
                three::cicp::TransformationEstimationPointToPoint4DoF(),
                three::ICPConvergenceCriteria());
        }

        if (result.correspondence_set_.size() > best_result.correspondence_set_.size()) {
            best_result = result;
        }
    }
    return best_result.transformation_;
}

void AnnotationTool(const folly::dynamic &config) {
    // EXTRACT PATHS
    std::string database_dir = config["CAD_database_root"].getString();
    std::string dataroot = config["dataroot"].getString();
    std::string dataset = config["dataset"].getString();
    std::string scene_dir = dataroot + "/" + dataset + "/";
    std::string fragment_dir = scene_dir + "/fragments/";
    bool debug_mode = config["debug"].asBool();
    // LOAD SCENE POINT CLOUD
    auto scene = std::make_shared<three::PointCloud>();
    three::ReadPointCloudFromPLY(scene_dir + "/test.klg.ply", *scene);
    if (debug_mode) three::DrawGeometries({scene}, "input scene");
    // LOAD FLOOR FRAGMENT
    auto floor = std::make_shared<three::PointCloud>();
    three::ReadPointCloudFromPLY(fragment_dir + "floor.ply", *floor);
    // COMPUTE ROTATION TO ALIGN Y AXIS IN CANONICAL FRAME
    auto floor_n = FindPlaneNormal(StdVectorOfEigenVectorToEigenMatrix(floor->points_));
    std::cout << "floor_n=" << floor_n.transpose() << "\n";
    auto R = RotationBetweenVectors(floor_n, {0, 1.0, 0});
    std::cout << "R=\n" << R << "\n";
    // CHECK Y AXIS ALIGNMENT
    Eigen::Matrix<double, 4, 4> T0;
    T0.block<3, 3>(0, 0) = R;
    T0(3, 3) = 1.0;
    std::cout << "T0=\n" << T0 << "\n";
    floor->Transform(T0);
    Eigen::Matrix4d T1;
    T1.setIdentity();
    T1.block<3, 1>(0, 3) = StdVectorOfEigenVectorMean(floor->points_);
    floor->Transform(T1);
    if (debug_mode) three::DrawGeometries({floor}, "gravity aligned floor");
    // READ OBJECT LIST
    std::string contents;
    folly::readFile(folly::sformat("{}/objects.json", fragment_dir).c_str(), contents);
    folly::dynamic obj_json = folly::parseJson(folly::json::stripComments(contents));
    // SETUP OUTPUT JSON FILE
    folly::dynamic out_json = folly::dynamic::object;
    for (auto each : obj_json["entries"]) {
        auto scan = std::make_shared<three::PointCloud>();
        auto model = std::make_shared<three::PointCloud>();

        // GET PROPER NAMES
        std::string scan_name = each.asString();
        std::string model_name = scan_name.substr(0, scan_name.find_last_of('_'));

        three::ReadPointCloudFromPLY(folly::sformat("{}/{}.ply", fragment_dir, scan_name), *scan);
        scan = three::VoxelDownSample(*scan, config["ICP"]["voxel_size"].asDouble());
        scan->Transform(T0);

        Eigen::Matrix4d T1;
        T1.setIdentity();
        auto mean = StdVectorOfEigenVectorMean(scan->points_);
        T1.block<3, 1>(0, 3) << -mean(0), -MinY(scan->points_), -mean(2);
        scan->Transform(T1);

        auto model_file = folly::sformat("{}/{}.obj", database_dir, model_name);
        std::cout << "model file @ " << model_file << "\n";
        Eigen::Matrix<double, Eigen::Dynamic, 3> V;
        Eigen::Matrix<int, Eigen::Dynamic, 3> F;
        igl::readOBJ(model_file, V, F);
        model->points_ = SamplePointCloudFromMesh(V, F, scan->points_.size() << 1);

        Eigen::Matrix4d T2;
        T2.setIdentity();
        mean = StdVectorOfEigenVectorMean(model->points_);
        T2.block<3, 1>(0, 3) << -mean(0), -MinY(model->points_), -mean(2);
        model->Transform(T2);

        std::cout << "#sampled points=" << model->points_.size() << "\n";
        if (debug_mode) {
            three::DrawGeometries({scan}, "partial scan");
            three::DrawGeometries({model}, "model");
        }


        Eigen::Matrix4d T3 = RegisterModelToScene(model, scan, config["ICP"]);

        // NOW LETS COMPOSE ALL THE TRANSFORMATIONS TOGETHER
        // MODEL - (T2, T3)-> ICP <-(T1, T0)- SCAN
        // MODEL -> SCENE:
        // (T1 * T0)^-1 * T3 * T2
        Eigen::Matrix4d Ttot = T1 * T0;
        Ttot.block<3, 3>(0, 0).transposeInPlace();
        Ttot.block<3, 1>(0, 3) = -Ttot.block<3, 3>(0, 0) * Ttot.block<3, 1>(0, 3);
        Ttot = Ttot * T3 * T2;

        // SAVE TOTAL TRANSFORMATION TO JSON FILE
        io::WriteMatrixToDynamic(out_json, scan_name, Ttot.block<3, 4>(0, 0));


        // CHECK THE TOTAL TRANSFORMATION
        three::ReadPointCloudFromPLY(folly::sformat("{}/{}.ply", fragment_dir, scan_name), *scan);
        scan->colors_.resize(scan->points_.size(), {0, 255, 0});
        igl::readOBJ(model_file, V, F);
        model->points_ = SamplePointCloudFromMesh(V, F, config["visualization"]["model_samples"].asInt());
        model->colors_.resize(model->points_.size(), {255, 0, 0});
        model->Transform(Ttot);
        if (debug_mode) {
            *scene += *model;
            *model += *scan;
            three::DrawGeometries({model});
        }
    }
    if (debug_mode) {
        three::DrawGeometries({scene});
    }
    std::cout << "Writing out overlaid scene for debugging ...\n";
    three::WritePointCloud(folly::sformat("{}/debug_scene.ply", fragment_dir), *scene, true);
    // SAVE OBJECT POSE AS JSON FILE
    folly::writeFile(folly::toPrettyJson(out_json), folly::sformat("{}/alignment.json", fragment_dir).c_str());
}


}   // namespace feh

