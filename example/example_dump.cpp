#include <memory>
#include <fstream>
#include <iostream>

// unix
#include "sys/stat.h"

// 3rdparty
#include "opencv2/imgproc/imgproc.hpp"

// feh
#include "dataloader.h"
#include "utils.h"

const int downsample_rate = 2;

using namespace feh;

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cout << TermColor::red
                  << "Usage: example_dump DIRECTORY_OF_THE_DATASET OUTPUT_DIRECTORY"
                  << TermColor::endl;
        exit(-1);
    }

    std::shared_ptr<VlslamDatasetLoader> loader;
    try {
        loader = std::make_shared<VlslamDatasetLoader>(argv[1]);
    } catch (const std::exception &) {
        std::cout << TermColor::red
                  << "failed to initialize dataset @ " << argv[1] << TermColor::endl;
        exit(-1);
    }

    // create directory
    const mode_t DIR_MODE = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
    if (mkdir(argv[2], DIR_MODE) < 0) {
        std::cout << TermColor::red
                  << "Failed to create output directory" << TermColor::endl;
        exit(-1);
    }

    auto ss = absl::StrFormat("%s/image", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << TermColor::red
                  << "Failed to create output/image directory" << TermColor::endl;
        exit(-1);
    }
    ss = absl::StrFormat("%s/pose", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << TermColor::red
                  << "Failed to create output/pose directory" << TermColor::endl;
        exit(-1);
    }
    ss = absl::StrFormat("%s/depth", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << TermColor::red
                  << "Failed to create output/depth directory" << TermColor::endl;
        exit(-1);
    }

    std::array<int, 2> size;
    std::vector<float> params;
    loader->GrabCameraInfo(size, params);
    for (auto each : params) {
        std::cout << each << " ";
    }

    Eigen::Matrix3f K;
    // parameters 
    K << params[0], 0, params[2],
      0, params[1], params[3],
      0, 0, 1;
    std::ofstream Kout(absl::StrFormat("%s/K.txt", argv[2]), std::ios::out);
    Kout << K;
    Kout.close();

    for (int i = 0; i < loader->size(); ++i) {
        SE3f gwc;
        SO3f Rg;    // rotation to align with gravity
        cv::Mat img, edgemap;
        vlslam_pb::BoundingBoxList bboxlist;

        loader->Grab(i, img, edgemap, bboxlist, gwc, Rg);   // grab datum
        auto depth_samples = loader->GrabSparseDepth(i);

        // std::cout << "g(world <- camera)=\n" << gwc.matrix3x4() << std::endl;
        // std::cout << "Rg=\n" << Rg.matrix() << std::endl;

        // write out pose
        std::ofstream fid_pose;
        fid_pose.open(absl::StrFormat("%s/pose/%06d.txt", argv[2], i));
        fid_pose << gwc.matrix();
        fid_pose.close();

        // write out sparse depth
        std::ofstream fid_depth;
        try {
            fid_depth.open(absl::StrFormat("%s/depth/%06d.txt", argv[2], i));
            for (const auto &s : depth_samples) {
                if (s.second[1] > 0) {
                    fid_depth << s.second[0] << " " 
                              << s.second[1] << " " 
                              << s.second[2] << std::endl;
                }
            }
            fid_depth.close();
        } catch (const std::exception &) {
            exit(-1);
        }

        // write out image
        cv::imwrite(absl::StrFormat("%s/image/%06d.jpg", argv[2], i), img);

        cv::imshow("image", img);
        // cv::imshow("edge map", edgemap);
        char ckey = cv::waitKey(30);
        if (ckey == 'q') break;
    }
}

