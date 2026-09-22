#include "../include/spectacularAI/unity/hand_tracking.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    const auto anchors = saiHandTracking::generatePalmAnchors(128, 128);
    assert(anchors.size() == 896);

    std::vector<float> scores(anchors.size(), -10.0f);
    std::vector<float> regressors(anchors.size() * 18, 0.0f);
    scores[0] = 10.0f;
    regressors[2] = 32.0f;
    regressors[3] = 32.0f;

    const auto detections = saiHandTracking::decodePalmDetections(
        scores,
        regressors,
        0.5f,
        true);
    assert(detections.size() == 1);
    assert(detections[0].score > 0.99f);
    assert(std::abs(detections[0].box[2] - 0.25f) < 0.0001f);
    assert(std::abs(detections[0].box[3] - 0.25f) < 0.0001f);

    std::cout << "Palm decoder test passed: " << anchors.size()
        << " anchors, " << detections.size() << " detection." << std::endl;
    return 0;
}