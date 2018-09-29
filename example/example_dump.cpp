#include <memory>
#include <fstream>

// unix
#include "sys/stat.h"

// 3rdparty
#include "folly/Format.h"
#include "opencv2/imgproc.hpp"

// feh
#include "dataloader.h"
#include "utils.h"

const int downsample_rate = 2;

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cout << feh::TermColor::red
                  << "Usage: example_dump DIRECTORY_OF_THE_DATASET OUTPUT_DIRECTORY"
                  << feh::TermColor::endl;
        exit(-1);
    }

    std::shared_ptr<feh::VlslamDatasetLoader> loader;
    try {
        loader = std::make_shared<feh::VlslamDatasetLoader>(argv[1]);
    } catch (const std::exception &) {
        std::cout << feh::TermColor::red
                  << "failed to initialize dataset @ " << argv[1] << feh::TermColor::endl;
        exit(-1);
    }

    // create directory
    const mode_t DIR_MODE = S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH;
    if (mkdir(argv[2], DIR_MODE) < 0) {
        std::cout << feh::TermColor::red
                  << "Failed to create output directory" << feh::TermColor::endl;
        exit(-1);
    }

    auto ss = folly::sformat("{}/image", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << feh::TermColor::red
                  << "Failed to create output/image directory" << feh::TermColor::endl;
        exit(-1);
    }
    ss = folly::sformat("{}/pose", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << feh::TermColor::red
                  << "Failed to create output/pose directory" << feh::TermColor::endl;
        exit(-1);
    }
    ss = folly::sformat("{}/depth", argv[2]);
    if (mkdir(ss.c_str(), DIR_MODE) < 0) {
        std::cout << feh::TermColor::red
                  << "Failed to create output/depth directory" << feh::TermColor::endl;
        exit(-1);
    }

    std::array<int, 2> size;
    std::vector<float> params;
    loader->GrabCameraInfo(size, params);
    for (auto each : params) {
        std::cout << each << " ";
    }

    cv::Mat prev_img, next_img;
    for (int i = 0; i < loader->size(); ++i) {
        if (i == 0 || i+1 == loader->size()) continue;
        cv::Mat img, edgemap;   // image and edge map
        vlslam_pb::BoundingBoxList bboxlist;    // list of bounding boxes
        Sophus::SE3f gwc, gwc1, gwc2;   // camera to world transformation
        Sophus::SO3f Rg;    // rotation to align with gravity

        loader->Grab(i, img, edgemap, bboxlist, gwc, Rg);   // grab datum
        loader->Grab(i-1, prev_img, edgemap, bboxlist, gwc1, Rg);   // grab datum
        loader->Grab(i+1, next_img, edgemap, bboxlist, gwc2, Rg);   // grab datum
        auto depth_samples = loader->GrabSparseDepth(i);

        int rows = img.rows / 2, cols = img.cols / 2;
        cv::Mat img_down{rows, cols, CV_8UC3}; 
        cv::Mat prev_img_down{rows, cols, CV_8UC3};
        cv::Mat next_img_down{rows, cols, CV_8UC3};
        cv::pyrDown(img, img_down);
        cv::pyrDown(prev_img, prev_img_down);
        cv::pyrDown(next_img, next_img_down);

        cv::Mat seq{rows, cols * 3, CV_8UC3};
        img_down.copyTo(seq(cv::Rect(cols, 0, cols, rows)));
        prev_img_down.copyTo(seq(cv::Rect(0, 0, cols, rows)));
        next_img_down.copyTo(seq(cv::Rect(cols*2, 0, cols, rows)));

        std::cout << "g(world <- camera)=\n" << gwc.matrix3x4() << std::endl;
        std::cout << "Rg=\n" << Rg.matrix() << std::endl;

        // write out pose
        std::ofstream fid_pose;
        try {
            auto g1c = gwc1.inverse() * gwc;     // current to previous one
            auto g2c = gwc2.inverse() * gwc;     // current to next one
            fid_pose.open(folly::sformat("{}/pose/{:06d}.txt", argv[2], i));
            fid_pose << g1c.matrix3x4();
            fid_pose << std::endl;
            fid_pose << g2c.matrix3x4();
            fid_pose.close();
        } catch (const std::exception &) {
            exit(-1);
        }

        // write out sparse depth
        std::ofstream fid_depth;
        try {
            fid_depth.open(folly::sformat("{}/depth/{:06d}.txt", argv[2], i));
            for (const auto &s : depth_samples) {
                fid_depth << s.second[0] / downsample_rate << " " 
                          << s.second[1] / downsample_rate << " " 
                          << s.second[2] << std::endl;
            }
            fid_depth.close();
        } catch (const std::exception &) {
            exit(-1);
        }

        cv::imshow("image", img_down);
        // cv::imshow("edge map", edgemap);
        cv::waitKey(30);
        // write out image
        cv::imwrite(folly::sformat("{}/image/{:06d}.jpg", argv[2], i), seq);

    }
}

