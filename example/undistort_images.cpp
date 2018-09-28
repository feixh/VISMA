//
// Undistort and crop images captured by Corvis' camera system to VGA size
// Created by visionlab on 2/7/18.
//
// stl
#include <memory>
#include <string>
#include <vector>

// 3rd party
#include "glog/logging.h"

// own
#include "undistorter.h"
#include "common/utils.h"


using namespace feh;


int main(int argc, char **argv) {
    float fx{0.561859}, fy{0.901540}, cx{0.491896}, cy{0.512629}, s{0.709402};
    int rows{600}, cols{960};
    int out_rows{500}, out_cols{960};
    auto undistorter = std::make_shared<UndistorterPTAM>(fx, fy, cx, cy, s,
                                                         rows, cols,
                                                         "crop",
                                                         100+out_rows, out_cols);

    const cv::Mat &K = undistorter->getK();
    fx = K.at<double>(0, 0);
    fy = K.at<double>(1, 1);
    cx = K.at<double>(2, 0);
    cy = K.at<double>(2, 1) - 50;
    CHECK_EQ(argc, 2);
    std::vector<std::string> imagefiles;
    Glob(argv[1], "png", imagefiles);
    for (auto imagefile : imagefiles) {
        std::cout << imagefile << "\n";
        cv::Mat img = cv::imread(imagefile);
        cv::Mat out;

        cv::copyMakeBorder(img, out, 50, 50, 0, 0, cv::BORDER_CONSTANT);
        std::vector<cv::Mat> slices(3), undistorted_slices(3);
        cv::split(out, slices);
        for (int i = 0; i < 3; ++i) {
            undistorter->undistort(slices[i], undistorted_slices[i]);
        }
        cv::merge(undistorted_slices, out);

        // crop top and bottom by 50 pixels
        cv::Mat final_out{out(cv::Rect(0, 50, out_cols, out_rows))};

        cv::imshow("distorted image", img);
        cv::imshow("un-distorted image", final_out);
        std::cout << "fx,fy,cx,cy,rows,cols=" << fx << " " << fy << " " << cx << " " << cy
                  << " " << undistorter->getOutputHeight()-100 << " " << undistorter->getOutputWidth() << "\n";
        cv::imwrite(imagefile, final_out);
        char ckey = cv::waitKey(24);
        if (ckey == 'q') break;
    }

}

