#include "renderer.h"
#include "utils.h"

using namespace feh;

int main(int argc, char **argv) {
  assert(argc > 1 && "usage: render_depth YOUR_CONFIGURATION.json");
  auto cfg = LoadJson(argv[1]);
  // get parameters
  int imH = cfg.get("image_height", 480).asInt();
  int imW = cfg.get("image_width", 640).asInt();
  float z_near = cfg.get("z_near", 0.05).asFloat();
  float z_far = cfg.get("z_far", 10.0).asFloat();
  float fx = cfg.get("fx", 240).asFloat();
  float fy = cfg.get("fy", 240).asFloat();
  float cx = cfg.get("cx", 320).asFloat();
  float cy = cfg.get("cy", 240).asFloat();
  // init to current camera transformation
  Mat4f g_curr_init = Mat4f::Identity();
  RendererPtr ptr = std::make_shared<Renderer>(imH, imW);
  // ptr->SetCamera(z_near, z_far, fx, fy, cx, fy);
  // ptr->SetCamera(g_curr_init);
}

