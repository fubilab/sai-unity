#pragma once

#include <array>
#include <vector>

namespace saiHandTracking {

struct PalmDetection {
    float score;
    std::array<float, 4> box;
    std::array<float, 14> keypoints;
};

std::vector<std::array<float, 4>> generatePalmAnchors(int inputWidth, int inputHeight);

std::vector<PalmDetection> decodePalmDetections(
    const std::vector<float>& scores,
    const std::vector<float>& regressors,
    float scoreThreshold,
    bool bestOnly);

}