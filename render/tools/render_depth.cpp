#include <iostream>

#include "renderer.h"
#include "utils.h"
#include "opencv2/highgui/highgui.hpp"

using namespace feh;

int main(int argc, char **argv) {
  assert(argc > 1 && "usage: render_depth YOUR_CONFIGURATION.json");
  auto cfg = LoadJson(argv[1]);
  // get parameters
  int imH = cfg.get("image_height", 480).asInt();
  int imW = cfg.get("image_width", 640).asInt();
  float z_near = cfg.get("z_near", 0.05).asFloat();
  float z_far = cfg.get("z_far", 5.0).asFloat();
  float fx = cfg.get("fx", 400).asFloat();
  float fy = cfg.get("fy", 400).asFloat();
  float cx = cfg.get("cx", 320).asFloat();
  float cy = cfg.get("cy", 240).asFloat();
  // init to current camera transformation
  Mat4f g_curr_init = Mat4f::Identity();
  std::cout << "g_curr_init=" << g_curr_init << std::endl;
  RendererPtr ptr = std::make_shared<Renderer>(imH, imW);
  ptr->SetCamera(z_near, z_far, fx, fy, cx, fy);
  ptr->SetCamera(g_curr_init);
  MatXf V;
  MatXi F;
  LoadMesh(cfg.get("mesh", "misc/truck.obj").asString(), V, F);
  // centering
  std::cout << "center=" << V.colwise().mean() << std::endl;
  std::cout << "max=" << V.colwise().maxCoeff() << std::endl;
  std::cout << "min=" << V.colwise().minCoeff() << std::endl;
  ptr->SetMesh(V, F);
  cv::Mat depth_map(imH, imW, CV_32FC1);
  cv::Mat mask(imH, imW, CV_8UC1);
  SE3f model_pose(SO3f(), GetVectorFromJson<float, 3>(cfg, "translation"));
  std::cout << model_pose.matrix() << std::endl;
  for (int i = 0; i < V.rows(); ++i) {
    Vec3f X{model_pose * V.row(i)};
    Vec2f x{(X(0) * fx + cx) / X(2), (X(1) * fy + cy) / X(2)};
    // std::cout << "X=" << X.transpose() << std::endl;
    std::cout << "x=" << x.transpose() << std::endl;
  }
  // ptr->RenderDepth(model_pose.matrix(), depth_map);
  ptr->RenderMask(model_pose.matrix(), mask);
  // cv::imshow("depth map", depth_map);
  cv::imshow("mask", mask);
  cv::waitKey();

  // TODO: this can be useful for other applications, move to utils?
  std::ofstream out("depth_map.bin", std::ios::out | std::ios::binary);
  assert(out.is_open());
  out.write((char*)&imH, sizeof imH);
  out.write((char*)&imW, sizeof imW);
  for (int i = 0; i < imH; ++i)
    for (int j = 0; j < imW; ++j) {
      float z = depth_map.at<float>(i, j);
      out.write((char*)&z, sizeof z);
    }
  out.close();
}

