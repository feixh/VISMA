#include <memory>
#include <fstream>

// unix
#include "sys/stat.h"

// 3rdparty
#include "folly/Format.h"

// feh
#include "dataset_loaders.h"
#include "common/utils.h"

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

    for (int i = 0; i < loader->size(); ++i) {
        cv::Mat img, edgemap;   // image and edge map
        vlslam_pb::BoundingBoxList bboxlist;    // list of bounding boxes
        Sophus::SE3f gwc;   // camera to world transformation
        Sophus::SO3f Rg;    // rotation to align with gravity
        loader->Grab(i, img, edgemap, bboxlist, gwc, Rg);   // grab datum
        auto depth_samples = loader->GrabSparseDepth(i);

        std::cout << "g(world <- camera)=\n" << gwc.matrix3x4() << std::endl;
        std::cout << "Rg=\n" << Rg.matrix() << std::endl;

        // print bounding boxes
        int j = 0;
        for (const auto &bbox : bboxlist.bounding_boxes()) {
            std::cout << "bbox #" << j++
                      << "(" << bbox.top_left_x() << "," << bbox.top_left_y() << ")-"
                      << "(" << bbox.bottom_right_x() << "," << bbox.bottom_right_y() << ")\n"
                      << "class=" << bbox.class_name() << "\n"
                      << "scores=";
            // Scores can store a list of scores of all the classes.
            // However, we only keep the score of the most likely class at index 0.
            for (const auto &score : bbox.scores()) std::cout << score << " ";
            std::cout << std::endl;
        }
        cv::imshow("image", img);
        cv::imshow("edge map", edgemap);
        cv::waitKey(30);

        // write out image
        cv::imwrite(folly::sformat("{}/image/{:06d}.jpg", argv[2], i), img);

        // write out pose
        std::ofstream fid_pose;
        try {
            fid_pose.open(folly::sformat("{}/pose/{:06d}.txt", argv[2], i));
            fid_pose << gwc.matrix3x4();
            fid_pose.close();
        } catch (const std::exception &) {
            exit(-1);
        }

        // write out sparse depth
        std::ofstream fid_depth;
        try {
            fid_depth.open(folly::sformat("{}/depth/{:06d}.txt", argv[2], i));
            for (const auto &s : depth_samples) {
                fid_depth << s.second[0] << " " << s.second[1] << " " << s.second[2] << std::endl;
            }
            fid_depth.close();
        } catch (const std::exception &) {
            exit(-1);
        }

    }
}

