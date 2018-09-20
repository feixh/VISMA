#pragma once

// stl
#include <vector>
#include <string>
#include <unordered_map>

// 3rd party
#include "sophus/se3.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

// own
#include "vlslam.pb.h"
#include "common/eigen_alias.h"

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
                      Sophus::SE3f &gwc,
                      Sophus::SO3f &Rg);
    /// \param fullpath: full path to the image file
    virtual bool Grab(int i,
                      cv::Mat &image,
                      cv::Mat &edgemap,
                      vlslam_pb::BoundingBoxList &bboxlist,
                      Sophus::SE3f &gwc,
                      Sophus::SO3f &Rg,
                      std::string &fullpath);
    std::unordered_map<int64_t, std::array<double, 6>> GrabPointCloud(int i, const cv::Mat &img);

    virtual int size() const { return size_; }
protected:
    std::string dataroot_;
    vlslam_pb::Dataset dataset_;
    std::vector<std::string> png_files_, edge_files_, bbox_files_;
    int size_;
};


}   // namespace feh
