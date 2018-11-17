#include "tool.h"

// open3d
#include "IO/IO.h"
#include "Visualization/Visualization.h"
// libigl
#include "igl/readOBJ.h"
#include "json/json.h"

// feh
#include "constrained_ICP.h"
#include "geometry.h"
#include "utils.h"

namespace feh {

void FindCorrespondence(const std::unordered_map<int, Model> &tgt,
                        const std::unordered_map<int, Model> &src,
                        const Eigen::Matrix<double, 4, 4> &T_tgt_src,
                        open3d::CorrespondenceSet &matches,
                        double threshold) {
    for (const auto &kv1 : src) {
        const Model &m1 = kv1.second;
        double min_dist = threshold;
        int best_match = -1;

        for (const auto &kv2 : tgt) {
            const Model &m2 = kv2.second;
            auto T_ef_model = T_tgt_src * m1.model_to_scene_;   // model -> ef
            // m2.model_to_scene_: model -> ef
            auto dT = T_ef_model.inverse() * m2.model_to_scene_;    // should close to identity
            if (dT.block<3, 1>(0, 3).norm() < min_dist) {
                min_dist = dT.block<3, 1>(0, 3).norm();
                best_match = kv2.first;
            }
        }
        if (best_match >= 0) {
            matches.push_back({kv1.first, best_match});
        }
    }
}

Eigen::Matrix4d OptimizeAlignment(
    const std::unordered_map<int, Model> &tgt,
    const std::unordered_map<int, Model> &src,
    const open3d::CorrespondenceSet &matches) {
  throw;
  // Need to implement log & exp map of SE(3) first
  /*
    std::vector<double> w(matches.size(), 1.0 / matches.size());
    Eigen::Matrix<float, 6, 1> sum, last_sum;
    int iter = 0;
    for (; iter < 100; ++iter) {
        sum.setZero();
        for (int k = 0; k < matches.size(); ++k) {
            const auto &match = matches[k];
            auto dT = tgt.at(match[1]).model_to_scene_ * src.at(match[0]).model_to_scene_.inverse();
            Eigen::Matrix<float, 6, 1> tangent = SE3f(dT.cast<float>()).log();
            sum += w[k] * tangent;
        }
        auto T = SE3d::exp(sum.cast<double>());
        // compute weights
        double sum_w(0);
        for (int k = 0; k < matches.size(); ++k) {
            const auto &match = matches[k];
            auto dT = tgt.at(match[1]).model_to_scene_ * (T.matrix() * src.at(match[0]).model_to_scene_).inverse();
            w[k] = 1.0 / std::max<double>(1e-4, SE3f(dT.cast<float>()).log().norm());
            sum_w += w[k];
        }
        for (auto &each_w : w) each_w /= sum_w;
        if ((last_sum - sum).norm() / sum.norm() < 1e-5) break;
        last_sum = sum;
    }
    std::cout << "Alignment optimization finished after " << iter << " iterations\n";
    return SE3d::exp(sum.cast<double>()).matrix();
    */
}

open3d::RegistrationResult RegisterScenes(
    const std::unordered_map<int, Model> &tgt,
    const std::unordered_map<int, Model> &src) {
    open3d::CorrespondenceSet best_matches;
    Eigen::Matrix4d best_T_tgt_src;
    best_T_tgt_src.setIdentity();

    for (const auto &kv1 : src) {
        const Model &m1 = kv1.second;
        std::cout << "model1.name=" << m1.model_name_ << "\n";
        for (const auto &kv2 : tgt) {
            const Model &m2 = kv2.second;
            std::cout << "model2.name=" << m2.model_name_ << "\n";
            if (m1.model_name_ == m2.model_name_) {
                // ONLY TEST WHEN THE TWO MODELS HAVE THE SAME SHAPE
                // SOURCE TO TARGET TRANSFORMATION
                auto T_tgt_src = m2.model_to_scene_ * m1.model_to_scene_.inverse(); // corvis -> elasticfusion (ef)
                std::cout << "T_tgt_src=\n" << T_tgt_src << "\n";
                // NOW LET'S CHECK THE RESIDUAL OF THIS PROPOSED TRANSFORMATION
                open3d::CorrespondenceSet matches;
                FindCorrespondence(tgt, src, T_tgt_src, matches, 0.5);
                if (matches.size() > best_matches.size()) {
                    best_matches = matches;
                    best_T_tgt_src = T_tgt_src;
                }
            }
        }
    }

    best_T_tgt_src = OptimizeAlignment(tgt, src, best_matches);
    open3d::RegistrationResult result(best_T_tgt_src);
    result.correspondence_set_ = best_matches;
    return result;
}

void MeshAlignment(const Json::Value &config) {
    // EXTRACT PATHS
    std::string database_dir = config["CAD_database_root"].asString();

    std::string dataroot = config["dataroot"].asString();
    std::string dataset = config["dataset"].asString();
    std::string scene_dir = dataroot + "/" + dataset + "/";
    std::string fragment_dir = scene_dir + "/fragments/";
    // LOAD SCENE POINT CLOUD
    auto scene = std::make_shared<open3d::PointCloud>();
    open3d::ReadPointCloudFromPLY(scene_dir + "/test.klg.ply", *scene);
    // READ GROUND TRUTH POSES
    std::string contents;
    auto gt_json = LoadJson(fragment_dir + "/alignment.json");

    // CONSTRUCT GROUND TRUTH UNORDERED_MAP
    std::unordered_map<int, Model> models;
    int counter(0);
    for (auto it = gt_json.begin(); it != gt_json.end(); ++it) {
        std::string key = it.key().asString();
        auto &this_model = models[counter];

        this_model.model_to_scene_.block<3, 4>(0, 0) = GetMatrixFromJson<double, 3, 4>(gt_json, key);
        this_model.model_name_ = key.substr(0, key.find_last_of('_'));
        std::cout << StrFormat("reading ... %s/%s.obj", database_dir, this_model.model_name_);
        Eigen::Matrix<double, Eigen::Dynamic, 6> tmp;
        igl::readOBJ(StrFormat("%s/%s.obj", database_dir, this_model.model_name_), tmp, this_model.F_);
        this_model.V_ = tmp.leftCols(3);

        std::shared_ptr <open3d::PointCloud> model_pc = std::make_shared<open3d::PointCloud>();
        model_pc->points_ = SamplePointCloudFromMesh(
            this_model.V_, this_model.F_, config["visualization"]["model_samples"].asInt());
        model_pc->colors_.resize(model_pc->points_.size(), {0, 255, 0});
        model_pc->Transform(this_model.model_to_scene_);    // TRANSFORM TO EF (ELASTICFUSION) FRAME
        this_model.pcd_ptr_ = model_pc;

        std::cout << "key=" << key << "\n" << models[counter].model_to_scene_ << "\n";
        ++counter;
    }

    // PUT OBJECTS IN THE SCENE ACCORDING TO GROUND TRUTH POSE
    if (config["evaluation"]["show_annotation"].asBool()) {
        for (const auto &key_val : models) {
            auto model = key_val.second;
            *scene += *(model.pcd_ptr_);
        }
        open3d::DrawGeometries({scene}, "Ground truth overlay");
    }

    // LOAD RESULT FILE
    std::string result_file = StrFormat("%s/result.json", scene_dir);
    std::cout << "result file=" << result_file << "\n";
    auto result = LoadJson(result_file);
    // ITERATE AND GET THE LAST ONE
    auto packet = result[result.size() - 1];
    auto scene_est = std::make_shared<open3d::PointCloud>();
    std::unordered_map<int, Model> models_est;
    for (const auto &obj : packet) {
        auto pose = GetMatrixFromJson<double, 3, 4>(obj, "model_pose");
        std::cout << StrFormat("id=%d\nstatus=%d\nshape=%s\npose=\n",
                                   obj["id"].asInt(),
                                   obj["status"].asInt(),
                                   obj["model_name"].asString())
                  << pose << "\n";

        auto &this_model = models_est[obj["id"].asInt()];
        this_model.model_name_ = obj["model_name"].asString();
        this_model.model_to_scene_.block<3, 4>(0, 0) = pose;
        Eigen::Matrix<double, Eigen::Dynamic, 6> tmp;
        igl::readOBJ(StrFormat("%s/%s.obj",
                                    database_dir,
                                    this_model.model_name_),
                     tmp, this_model.F_);
        this_model.V_ = tmp.leftCols(3);

        std::shared_ptr <open3d::PointCloud> model_pc = std::make_shared<open3d::PointCloud>();
        model_pc->points_ = SamplePointCloudFromMesh(
            this_model.V_, this_model.F_,
            config["visualization"]["model_samples"].asInt());
        model_pc->colors_.resize(model_pc->points_.size(), {255, 0, 0});
        model_pc->Transform(this_model.model_to_scene_);    // ALREADY IN CORVIS FRAME
        this_model.pcd_ptr_ = model_pc;

        *scene_est += *model_pc;
    }

    open3d::DrawGeometries({scene_est}, "semantic reconstruction");
    auto ret = RegisterScenes(models, models_est);
    auto T_ef_corvis = ret.transformation_;
    std::cout << "T_ef_corvis=\n" << T_ef_corvis << "\n";
    for (int i = 0; i < ret.correspondence_set_.size(); ++i) {
        std::cout << StrFormat("%d-%d\n", ret.correspondence_set_[i][0], ret.correspondence_set_[i][1]);
    }

    if (config["evaluation"]["ICP_refinement"].asBool()) {
        // RE-LOAD THE SCENE
        std::shared_ptr<open3d::PointCloud> raw_scene = std::make_shared<open3d::PointCloud>();
        open3d::ReadPointCloudFromPLY(scene_dir + "/test.klg.ply", *raw_scene);
        // FIXME: MIGHT NEED CROP THE 3D REGION-OF-INTEREST HERE
        auto result = ICPRefinement(raw_scene,
                                    models_est,
                                    T_ef_corvis,
                                    config["evaluation"]);
        T_ef_corvis = result.transformation_;
    }
    // // save the alignment
    // folly::dynamic out = folly::dynamic::object();
    // WriteMatrixToJson(out, "T_ef_corvis", T_ef_corvis.block<3, 4>(0, 0));
    // std::string output_path = scene_dir + "/result_alignment.json";
    // folly::writeFile(folly::toPrettyJson(out), output_path.c_str());
    Json::Value out;
    WriteMatrixToJson(out, "T_ef_corvis", T_ef_corvis.block<3, 4>(0, 0));
    std::string output_path = scene_dir + "/result_alignment.json";
    std::ofstream json_out(output_path, std::ios::out);
    assert(json_out.is_open());
    json_out << out;
    std::cout << "T_ef_corvis written to " << output_path << "\n";

//    open3d::ReadPointCloudFromPLY(config["scene_directory"].asString() + "/test.klg.ply", *scene);
    // NOW LETS LOOK AT THE ESTIMATED SCENE IN RGB-D SCENE FRAME
    for (const auto &kv : models_est) {
        const auto &this_model = kv.second;
        this_model.pcd_ptr_->Transform(T_ef_corvis);
        *scene += *(this_model.pcd_ptr_);
    }
    open3d::DrawGeometries({scene}, "semantic reconstruction aligned to RGB-D");
    open3d::WritePointCloud(scene_dir+"/augmented_view.ply", *scene);
}


open3d::RegistrationResult ICPRefinement(std::shared_ptr<open3d::PointCloud> scene,
                                        const std::unordered_map<int, Model> &src,
                                        const Eigen::Matrix4d &T_scene_src,
                                        const Json::Value &options) {
    // CONSTRUCT ESTIMATED SCENE
    auto scene_est = std::make_shared<open3d::PointCloud>();
    for (const auto &kv : src) {
        const auto &this_model = kv.second;
        auto model_ptr = std::make_shared<open3d::PointCloud>();
        model_ptr->points_ = SamplePointCloudFromMesh(this_model.V_, this_model.F_, options["samples_per_model"].asInt());
        model_ptr->Transform(this_model.model_to_scene_);
        *scene_est += *model_ptr;
    }

    scene = open3d::VoxelDownSample(*scene, options.get("voxel_size", 0.02).asDouble());
    open3d::RegistrationResult result;
    if (options["use_point_to_plane"].asBool()) {
        result = open3d::RegistrationICP(*scene_est,
                                        *scene,
                                        options.get("max_distance", 0.05).asDouble(),
                                        T_scene_src,
                                        open3d::TransformationEstimationPointToPlane());
    } else {
        result = open3d::RegistrationICP(*scene_est,
                                        *scene,
                                        options.get("max_distance", 0.05).asDouble(),
                                        T_scene_src);
    }
    std::cout << StrFormat("fitness=%f; inlier_rmse=%f\n", result.fitness_, result.inlier_rmse_);
    return result;
}

void QuantitativeEvaluation(Json::Value config) {
    // disable original mesh
    // CHECK(!config["result_visualization"]["show_original_scene"].getBool());

    // Align semantic mapping and RGB-D reconstruction first.
    MeshAlignment(config);

    // assemble result scene mesh
    Eigen::Matrix<double, Eigen::Dynamic, 6> tmp;

    Eigen::Matrix<double, Eigen::Dynamic, 3> Vr;
    Eigen::Matrix<int, Eigen::Dynamic, 3> Fr;
    std::vector<Eigen::Matrix<double, 3, 4>> Gr;
    AssembleResult(config, &Vr, &Fr, &Gr);
    std::cout << TermColor::cyan << "Result scene mesh assembled" << TermColor::endl;

    // assemble ground truth scene mesh
    Eigen::Matrix<double, Eigen::Dynamic, 3> Vg;
    Eigen::Matrix<int, Eigen::Dynamic, 3> Fg;
    std::vector<Eigen::Matrix<double, 3, 4>> Gg; // ground truth poses
    AssembleGroundTruth(config, &Vg, &Fg, &Gg);
    std::cout << TermColor::cyan << "Ground truth scene mesh assembled" << TermColor::endl;

    // debug
    for (int i = 0; i < Gr.size(); ++i) {
        std::cout << "Gr[" << i << "]=\n" << Gr[i] << "\n";
    }
    for (int i = 0; i < Gg.size(); ++i) {
        std::cout << "Gg[" << i << "]=\n" << Gg[i] << "\n";
    }

    std::cout << TermColor::cyan << "Computing pose error ..." << TermColor::endl;
    // searching NN within threshold, and compute the average distance
    Json::Value pose_error_cfg;
    pose_error_cfg["dist_thresh"] = 0.5;
    auto pose_stats = MeasurePoseError(Gr, Gg, pose_error_cfg);
    std::cout << "translation errors:\n";
    PrintErrorMetric(pose_stats[0]);
    std::cout << "rotation errors:\n";
    // convert rotation from rad to degree
    pose_stats[1].mean_ *= 180 / 3.14;
    pose_stats[1].median_ *= 180 / 3.14;
    pose_stats[1].min_ *= 180 / 3.14;
    pose_stats[1].max_ *= 180 / 3.14;
    pose_stats[1].std_ *= 180 / 3.14;
    PrintErrorMetric(pose_stats[1]);

    // measure surface error
    std::cout << TermColor::cyan << "Computing surface error ..." << TermColor::endl;
    Json::Value surface_error_cfg;
    surface_error_cfg["num_samples"] = std::min<uint64_t>(500000, Fg.rows()*100);
    auto stats = MeasureSurfaceError(Vr, Fr, Vg, Fg, surface_error_cfg);
    std::cout << "surface errors:\n";
    PrintErrorMetric(stats);

    auto save_metric = [] (std::string filename, const GenericErrorMetric<double>& metric) {
        // folly::dynamic out_json = folly::dynamic::object
        //     ("mean", metric.mean_)
        //     ("std", metric.std_)
        //     ("min", metric.min_)
        //     ("max", metric.max_)
        //     ("median", metric.median_);
        // folly::writeFile(folly::toPrettyJson(out_json), filename.c_str());
      Json::Value out_json;
      out_json["mean"] = metric.mean_;
      out_json["std"] = metric.std_;
      out_json["min"] = metric.min_;
      out_json["max"] = metric.max_;
      out_json["median"] = metric.median_;
      SaveJson(out_json, filename);
    };

    // write out result
    std::string error_filename = StrFormat("%s/%s/surface_error.json",
                                                config["dataroot"].asString(),
                                                config["dataset"].asString());
    save_metric(error_filename, stats);

    error_filename = StrFormat("%s/%s/translation_error.json",
                                    config["dataroot"].asString(),
                                    config["dataset"].asString());
    save_metric(error_filename, pose_stats[0]);

    error_filename = StrFormat("%s/%s/rotation_error.json",
                                    config["dataroot"].asString(),
                                    config["dataset"].asString());
    save_metric(error_filename, pose_stats[1]);

}

}
