#pragma once

#include <array>
#include <vector>

namespace saiHandTracking {

struct PalmDetection {
    float score;
    std::array<float, 4> box;
    std::array<float, 14> keypoints;
    float landmarkScore = 0.0f;
    float handedness = 0.5f;
    std::array<float, 63> landmarks{};
    std::array<float, 63> worldLandmarks{};
    int gesture = -1;
    bool hasLandmarks = false;
};

struct LandmarkRoi {
    float centerX;
    float centerY;
    float width;
    float height;
    float angleDegrees;
};

std::vector<std::array<float, 4>> generatePalmAnchors(int inputWidth, int inputHeight);

std::vector<PalmDetection> decodePalmDetections(
    const std::vector<float>& scores,
    const std::vector<float>& regressors,
    float scoreThreshold,
    bool bestOnly);

std::vector<PalmDetection> suppressPalmDetections(
    std::vector<PalmDetection> detections,
    float iouThreshold,
    std::size_t maxDetections);

LandmarkRoi calculateLandmarkRoi(
    const PalmDetection& detection,
    float imageWidth,
    float imageHeight);

void decodeLandmark(
    PalmDetection& detection,
    const std::vector<float>& score,
    const std::vector<float>& handedness,
    const std::vector<float>& landmarks,
    const std::vector<float>& worldLandmarks,
    const LandmarkRoi& roi,
    float imageWidth,
    float imageHeight);

}