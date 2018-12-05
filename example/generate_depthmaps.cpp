#include "utils.h"
#include "renderer.h"
#include "dataloader.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace feh;

const int imH = 500;
const int imW = 960;
const float fx = 486.405;
const float fy = 535.401;
const float cx = 469.199;
const float cy = 257.916;
const float z_near = 0.05;
const float z_far = 5.0;
const float MAX_DEPTH = 5.0;

bool LoadXYZ(const std::string &filename, MatXf &V) {
  try {
    std::ifstream infile(filename, std::ios::in);
    if (!infile.is_open()) return false;
    float x, y, z;
    std::vector<Vec3f> tmp;
    while (infile >> x >> y >> z) {
      tmp.emplace_back(x, y, z);
    }
    infile.close();
    V = StdVectorOfEigenVectorToEigenMatrix(tmp);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void DepthFromPointcloud(const MatXf &V, const Mat3f &K, const SE3f &g, cv::Mat &depth) {
  depth.setTo(MAX_DEPTH);
  for (int i = 0; i < V.rows(); ++i) {
    // Vec3f X = K * (g * Vec3f{V.row(i)});
    Vec3f X = K * (g * Vec3f{V.row(i)});
    // std::cout << "X=" << X.transpose() << std::endl;
    if (X(2) > 0) {
      Vec2i xc{int(X(0) / X(2)), int(X(1) / X(2))};
      // std::cout << StrFormat("[x,y,Z]=%f, %f, %f\n", xc(0), xc(1), X(2));
      if (xc(0) >= 0 && xc(0) < depth.cols 
          && xc(1) >= 0 && xc(1) < depth.rows
          && depth.at<float>(xc(1), xc(0)) > X(2)) {
          depth.at<float>(xc(1), xc(0)) = X(2);
      }
    }
  }
  // Perform min filtering on image using erode
  cv::erode(depth, depth, cv::Mat{});
}

int main(int argc, char **argv) {
  std::string model_path = argv[1];
  std::string alignment_path = argv[2];
  std::string dataset_dir = argv[3];

  // load point cloud
  MatXf V; 
  if (!LoadXYZ(model_path, V)) {
    std::cout << "failed to open " << model_path << std::endl; 
    exit(-1);
  }
  std::cout << StrFormat("V.rows=%d, V.cols=%d\n", V.rows(), V.cols());
  std::cout << "V.mean=" << V.colwise().mean().transpose() << std::endl;


  // setup intrinsics
  Mat3f K;
  K << fx, 0, cx,
    0, fy, cy,
    0, 0, 1;

  // load alignment
  auto alignment_data = GetMatrixFromJson<float, 3, 4> (
      LoadJson(alignment_path), "T_ef_corvis");
  SE3 g_ef_corvis(SO3(alignment_data.block<3, 3>(0, 0)), 
      alignment_data.block<3, 1>(0, 3));
  std::cout << "g_ef_corvis=\n" << g_ef_corvis.matrix() << std::endl;
  std::cout << "g_corvis_ef=\n" << g_ef_corvis.inv().matrix() << std::endl;

  // load dataset
  std::shared_ptr<VlslamDatasetLoader> loader;
  try {
      loader = std::make_shared<VlslamDatasetLoader>(dataset_dir);
  } catch (const std::exception &) {
      std::cout << TermColor::red
                << "Usage: example_load DIRECTORY_OF_THE_DATASET" << TermColor::endl;
      exit(-1);
  }
  for (int i = 0; i < loader->size(); ++i) {
    cv::Mat img, edgemap;   // image and edge map
    vlslam_pb::BoundingBoxList bboxlist;    // list of bounding boxes
    SE3f gwc;   // camera to world transformation
    SO3f Rg;    // rotation to align with gravity
    std::string fullpath;
    loader->Grab(i, img, edgemap, bboxlist, gwc, Rg, fullpath);   // grab datum

    // std::cout << "g(world <- camera)=\n" << gwc.matrix3x4() << std::endl;
    // std::cout << "Rg=\n" << Rg.matrix() << std::endl;
    //
    // model is fixed, everytime we update the camera pose
    cv::Mat depthmap(imH, imW, CV_32FC1);
    std::cout << "rendering ..." << std::endl;
    DepthFromPointcloud(V, K, gwc.inv() * g_ef_corvis.inv(), depthmap);
    std::string depth_path = fullpath.substr(0, fullpath.size()-4) + ".depth";
    std::cout << "saving depth to " << depth_path << std::endl;
    SaveMat<float>(depth_path, depthmap);

    if (!img.empty()) cv::imshow("image", img);
    cv::imshow("depth", depthmap);
    if (cv::waitKey(1) == 'q') break;
  }

}
