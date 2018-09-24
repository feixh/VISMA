#include <memory>
#include "dataset_loaders.h"
#include "common/utils.h"

int main(int argc, char *argv[]) {
    std::shared_ptr<feh::VlslamDatasetLoader> loader;
    try {
        loader = std::make_shared<feh::VlslamDatasetLoader>(argv[1]);
    } catch (const std::exception &) {
        std::cout << feh::TermColor::red
                  << "Usage: example_loader DIRECTORY_OF_THE_DATASET" << feh::TermColor::endl;
        exit(-1);
    }
    for (int i = 0; i < loader->size(); ++i) {
        cv::Mat img, edgemap;   // image and edge map
        vlslam_pb::BoundingBoxList bboxlist;    // list of bounding boxes
        Sophus::SE3f gwc;   // camera to world transformation
        Sophus::SO3f Rg;    // rotation to align with gravity
        loader->Grab(i, img, edgemap, bboxlist, gwc, Rg);   // grab datum

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
    }
}
