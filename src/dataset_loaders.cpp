#include "dataset_loaders.h"

// 3rd party
#include "folly/FileUtil.h"
#include "folly/dynamic.h"
#include "folly/json.h"

// own
#include "io_utils.h"
#include "opencv2/imgproc.hpp"

namespace feh {


VlslamDatasetLoader::VlslamDatasetLoader(const std::string &dataroot):
dataroot_(dataroot) {

    std::ifstream in_file(dataroot_ + "/dataset");
    CHECK(in_file.is_open()) << "failed to open dataset";

    dataset_.ParseFromIstream(&in_file);
    in_file.close();

    if (!feh::Glob(dataroot_, ".png", png_files_)) {
        LOG(FATAL) << "FATAL::failed to read png file list @" << dataroot_;
    }

//    for (int i = 0; i < png_files_.size(); ++i) {
//        std::cout << png_files_[i] << "\n";
//    }


    if (!feh::Glob(dataroot_, ".edge", edge_files_)) {
        LOG(FATAL) << "FATAL::failed to read edge map list @" << dataroot_;
    }

    if (!feh::Glob(dataroot_, ".bbox", bbox_files_)) {
        LOG(FATAL) << "FATAL::failed to read bounding box lisst @" << dataroot_;
    }

    CHECK_EQ(png_files_.size(), edge_files_.size());
    CHECK_EQ(png_files_.size(), bbox_files_.size());
    size_ = png_files_.size();
}


bool VlslamDatasetLoader::Grab(int i,
                               cv::Mat &image,
                               cv::Mat &edgemap,
                               vlslam_pb::BoundingBoxList &bboxlist,
                               Sophus::SE3f &gwc,
                               Sophus::SO3f &Rg,
                               std::string &fullpath) {
    fullpath = png_files_[i];
    return Grab(i, image, edgemap, bboxlist, gwc, Rg);
}

bool VlslamDatasetLoader::Grab(int i,
                               cv::Mat &image,
                               cv::Mat &edgemap,
                               vlslam_pb::BoundingBoxList &bboxlist,
                               Sophus::SE3f &gwc,
                               Sophus::SO3f &Rg) {

    if (i >= size_ || i < 0) return false;
    std::cout << i << "\n";

    vlslam_pb::Packet *packet_ptr(dataset_.mutable_packets(i));
    gwc = Sophus::SE3f(feh::io::SE3FromArray(packet_ptr->mutable_gwc()->mutable_data()));

    // gravity alignment rotation
    feh::Vec3f Wg(packet_ptr->wg(0), packet_ptr->wg(1), 0);
    Rg = Sophus::SO3f::exp(Wg);

    std::string png_file = png_files_[i];
    std::string edge_file = edge_files_[i];
    std::string bbox_file = bbox_files_[i];

    // read image
    image = cv::imread(png_file);
    CHECK(!image.empty()) << "empty image: " << png_file;

    // read edgemap
    if (!feh::io::LoadEdgeMap(edge_file, edgemap)) {
        LOG(FATAL) << "failed to load edge map @ " << edge_file;
    }

    // read bounding box
    std::ifstream in_file(bbox_file, std::ios::in);
    CHECK(in_file.is_open()) << "FATAL::failed to open bbox file @ " << bbox_file;
    bboxlist.ParseFromIstream(&in_file);
    in_file.close();
    return true;
}

std::unordered_map<int64_t, std::array<double, 6>> VlslamDatasetLoader::GrabPointCloud(int i,
                                                                                  const cv::Mat &img) {
    std::unordered_map<int64_t, std::array<double, 6>> out;
    vlslam_pb::Packet *packet_ptr = dataset_.mutable_packets(i);
    for (auto f : packet_ptr->features()) {
        if (f.status() == vlslam_pb::Feature_Status_INSTATE
            || f.status() == vlslam_pb::Feature_Status_GOODDROP) {
            auto color = img.at<cv::Vec3b>(int(f.xp(1)), int(f.xp(0)));
            if (out.count(f.id())) {
                color[0] += out.at(f.id())[3];
                color[1] += out.at(f.id())[4];
                color[2] += out.at(f.id())[5];
                color[0] >>= 1;
                color[1] >>= 1;
                color[2] >>= 1;
                out[f.id()] = {f.xw(0), f.xw(1), f.xw(2),
                               static_cast<double>(color[0]),
                               static_cast<double>(color[1]),
                               static_cast<double>(color[2])};
            } else {
                out[f.id()] = {f.xw(0), f.xw(1), f.xw(2),
                               static_cast<double>(color[0]),
                               static_cast<double>(color[1]),
                               static_cast<double>(color[2])};
            }
        }
    }
    return out;
};


}   // namespace feh
