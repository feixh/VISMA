#include "utils.h"
#include "tool.h"

using namespace feh;

int main(int argc, char **argv) {
    // READ IN CONFIGURATION
    auto config = LoadJson("../cfg/tool.json");
    // TURN OFF ORIGINAL SCENE MESH IN QUANTITATIVE EVALUATION
    // AND ONLY COMPARE MESHES CONSIST OF OBJECTS-OF-INTEREST
    // config["result_visualization"]["show_original_scene"] = false;
    QuantitativeEvaluation(config);
}


