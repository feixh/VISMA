// folly
#include "folly/dynamic.h"
#include "folly/json.h"
#include "folly/FileUtil.h"
// igl
#include "tool.h"

using namespace feh;

int main(int argc, char **argv) {
    // READ IN CONFIGURATION
    folly::fbstring contents;
    folly::readFile("../cfg/tool.json", contents);
    folly::dynamic config = folly::parseJson(folly::json::stripComments(contents));
    // TURN OFF ORIGINAL SCENE MESH IN QUANTITATIVE EVALUATION
    // AND ONLY COMPARE MESHES CONSIST OF OBJECTS-OF-INTEREST
    // config["result_visualization"]["show_original_scene"] = false;
    QuantitativeEvaluation(config);
}


