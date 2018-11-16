#pragma once

// stl
#include <vector>
#include <string>
#include <unordered_map>

// 3rd party
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

// own
#include "vlslam.pb.h"
#include "alias.h"

namespace feh {

class VlslamDatasetLoader {
public:
    VlslamDatasetLoader() {}
    VlslamDatasetLoader(const std::string &dataroot);
    /// \brief: Grab datum at index i.
    /// \param i: index
    /// \param image:
    /// \param edgemap:
    /// \param bboxlist:
    /// \param gwc: camera to world transformation
    /// \param Rg: gravity rotation
    virtual bool Grab(int i,
                      cv::Mat &image,
                      cv::Mat &edgemap,
                      vlslam_pb::BoundingBoxList &bboxlist,
                      SE3f &gwc,
                      SO3f &Rg);
    /// \param fullpath: full path to the image file
    virtual bool Grab(int i,
                      cv::Mat &image,
                      cv::Mat &edgemap,
                      vlslam_pb::BoundingBoxList &bboxlist,
                      SE3f &gwc,
                      SO3f &Rg,
                      std::string &fullpath);
    std::unordered_map<int64_t, std::array<ftype, 6>> GrabPointCloud(int i, const cv::Mat &img);
    std::unordered_map<int64_t, std::array<ftype, 3>> GrabSparseDepth(int i);
    /// \brief: Get camera information.
    /// \returns: [rows, cols] size of the image
    ///           [fx, fy, cx, cy]
    void GrabCameraInfo(std::array<int, 2> &size, std::vector<float> &params) {
        size = {dataset_.camera().rows(), dataset_.camera().cols()};    // height, width
        for (int i = 0; i < dataset_.camera().parameters_size(); ++i)  {
            params.push_back(dataset_.camera().parameters(i));
        }
    }

    virtual int size() const { return size_; }
protected:
    std::string dataroot_;
    vlslam_pb::Dataset dataset_;
    std::vector<std::string> png_files_, edge_files_, bbox_files_;
    int size_;
};


}   // namespace feh
