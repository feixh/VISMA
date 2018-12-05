#include <iostream>
#include <string>

#include "renderer.h"
#include "utils.h"
#include "opencv2/highgui/highgui.hpp"

using namespace feh;


int main(int argc, char **argv) {
  assert(argc > 1 && "usage: render_depth YOUR_CONFIGURATION.json");
  auto cfg = LoadJson(argv[1]);
  // get opengl version
  int major_version = cfg.get("major_version", 4).asInt();
  int minor_version = cfg.get("minor_version", 3).asInt();
  // get parameters
  int imH = cfg.get("image_height", 480).asInt();
  int imW = cfg.get("image_width", 640).asInt();
  float z_near = cfg.get("z_near", 0.05).asFloat();
  float z_far = cfg.get("z_far", 10.0).asFloat();
  float fx = cfg.get("fx", 400).asFloat();
  float fy = cfg.get("fy", 400).asFloat();
  float cx = cfg.get("cx", 320).asFloat();
  float cy = cfg.get("cy", 240).asFloat();
  // init to current camera transformation
  Mat4f g_curr_init = Mat4f::Identity();
  std::cout << "g_curr_init=" << g_curr_init << std::endl;

  RendererPtr ptr = std::make_shared<Renderer>(imH, imW, major_version, minor_version);
  ptr->SetCamera(z_near, z_far, fx, fy, cx, fy);
  ptr->SetCamera(g_curr_init);
  MatXf V;
  MatXi F;
  LoadMesh(cfg.get("mesh", "misc/hermanmiller_aeron.obj").asString(), V, F);
  ptr->SetMesh(V, F);

  // stats of mesh
  std::cout << "center=" << V.colwise().mean() << std::endl;
  std::cout << "max=" << V.colwise().maxCoeff() << std::endl;
  std::cout << "min=" << V.colwise().minCoeff() << std::endl;


  SE3f model_pose(SO3f(), GetVectorFromJson<float, 3>(cfg, "translation"));

  cv::Mat depth_map(imH, imW, CV_32FC1);
  ptr->RenderDepth(model_pose.matrix(), depth_map);

  cv::Mat mask(imH, imW, CV_8UC1);
  bool has_mask = cfg.get("mask", false).asBool();
  if (has_mask) {
    ptr->RenderMask(model_pose.matrix(), mask);
  }

  if (cfg.get("show", false).asBool()) {
    cv::imshow("depth map", depth_map);
    if (has_mask) {
      cv::imshow("depth map", mask);
    }
    cv::waitKey();
  }

  if (cfg.get("save", false).asBool()) {
    std::string output_path = cfg.get("output_path", ".").asString();
    try {
      SaveMat<float>(output_path+"/depthmap.bin", depth_map);
    } catch (const std::exception &) {
      std::cout << "failed to write out depth map to " << output_path << std::endl;
    }

    if (has_mask) {
      try {
        SaveMat<uint8_t>(output_path+"/mask.bin", mask);
      } catch (const std::exception &) {
        std::cout << "failed to write out mask to " << output_path << std::endl;
      }
    }
  }
}

