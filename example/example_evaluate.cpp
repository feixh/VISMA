#include "common/eigen_alias.h"

// stl
#include <chrono>
#include <sophus/se3.hpp>
// folly
#include "folly/dynamic.h"
#include "folly/json.h"
#include "folly/FileUtil.h"
// igl
#include "igl/readOBJ.h"
// feh
#include "io_utils.h"
#include "evaluation.h"

using namespace feh;

int main(int argc, char **argv) {
//    if (argc != 2 && argc != 3) {
//        std::cout << "USAGE:\n tool OPTION [DATASET]\n OPTION=a|e\n a for annotation\n e for evaluation\n DATASET=dataset to evaluate";
//        exit(-1);
//    }
    // READ IN CONFIGURATION
    folly::fbstring contents;
    folly::readFile("../cfg/tool.json", contents);
    folly::dynamic config = folly::parseJson(folly::json::stripComments(contents));
    bool failed = false;
    if (argc >= 3) {
        config["dataset"] = std::string(argv[2]);
    }
    if (argv[1][0] == 'a') {
        AnnotationTool(config);
    } else if (argv[1][0] == 'e') {
        EvaluationTool(config);
    } else if (argv[1][0] == 'v') {
        if (argv[1][1] == 'g') {
            AssembleGroundTruth(config);
        } else if (argv[1][1] == 'r') {
            VisualizeResult(config);
//            AssembleResult(config);
        } else {
            failed = true;
        }
    } else if (argv[1][0] == 'q') {
        // TURN OFF ORIGINAL SCENE MESH IN QUANTITATIVE EVALUATION
        // AND ONLY COMPARE MESHES CONSIST OF OBJECTS-OF-INTEREST
        config["result_visualization"]["show_original_scene"] = false;
        QuantitativeEvaluation(config);
    } else {
        failed = true;
    }

    if (failed) {
        std::cout << "USAGE:\n tool OPTION\n OPTION=[a|e|v(g|r)|q|i]\n"
            "a for annotation\n"
            "e for evaluation\n"
            "vg for ground truth visualization\n"
            "vr for results visualization\n"
            "q for quantitative evaluation\n";
        exit(-1);
    }
}

