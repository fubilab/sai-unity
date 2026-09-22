#include "../include/spectacularAI/unity/hand_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace saiHandTracking {
namespace {

float calculateScale(float minScale, float maxScale, int strideIndex, int strideCount) {
    if (strideCount == 1) return (minScale + maxScale) / 2.0f;
    return minScale + (maxScale - minScale) * strideIndex / (strideCount - 1);
}

}

std::vector<std::array<float, 4>> generatePalmAnchors(int inputWidth, int inputHeight) {
    constexpr float minScale = 0.1484375f;
    constexpr float maxScale = 0.75f;
    const std::array<int, 4> strides = {8, 16, 16, 16};
    std::vector<std::array<float, 4>> anchors;

    int layerIndex = 0;
    while (layerIndex < static_cast<int>(strides.size())) {
        int lastSameStrideLayer = layerIndex;
        int anchorsPerCell = 0;
        while (lastSameStrideLayer < static_cast<int>(strides.size()) &&
               strides[lastSameStrideLayer] == strides[layerIndex]) {
            anchorsPerCell += 2;
            ++lastSameStrideLayer;
        }

        int featureMapHeight = (inputHeight + strides[layerIndex] - 1) / strides[layerIndex];
        int featureMapWidth = (inputWidth + strides[layerIndex] - 1) / strides[layerIndex];
        for (int y = 0; y < featureMapHeight; ++y) {
            for (int x = 0; x < featureMapWidth; ++x) {
                for (int anchorIndex = 0; anchorIndex < anchorsPerCell; ++anchorIndex) {
                    anchors.push_back({
                        (x + 0.5f) / featureMapWidth,
                        (y + 0.5f) / featureMapHeight,
                        1.0f,
                        1.0f});
                }
            }
        }

        layerIndex = lastSameStrideLayer;
    }

    return anchors;
}

std::vector<PalmDetection> decodePalmDetections(
    const std::vector<float>& scores,
    const std::vector<float>& regressors,
    float scoreThreshold,
    bool bestOnly) {
    constexpr int valuesPerDetection = 18;
    constexpr float scale = 128.0f;
    const std::vector<std::array<float, 4>> anchors = generatePalmAnchors(128, 128);
    const std::size_t detectionCount = std::min({
        anchors.size(),
        scores.size(),
        regressors.size() / valuesPerDetection});

    std::size_t bestIndex = detectionCount;
    float bestScore = scoreThreshold;
    if (bestOnly) {
        for (std::size_t index = 0; index < detectionCount; ++index) {
            float score = 1.0f / (1.0f + std::exp(-scores[index]));
            if (score > bestScore) {
                bestScore = score;
                bestIndex = index;
            }
        }
    }

    std::vector<PalmDetection> detections;
    for (std::size_t index = 0; index < detectionCount; ++index) {
        if (bestOnly && index != bestIndex) continue;

        float score = 1.0f / (1.0f + std::exp(-scores[index]));
        if (score < scoreThreshold) continue;

        const auto& anchor = anchors[index];
        const float* regression = regressors.data() + index * valuesPerDetection;
        std::array<float, valuesPerDetection> decoded;
        for (int valueIndex = 0; valueIndex < valuesPerDetection; ++valueIndex) {
            float anchorCoordinate = valueIndex % 2 == 0 ? anchor[0] : anchor[1];
            decoded[valueIndex] = regression[valueIndex] / scale + anchorCoordinate;
        }

        decoded[2] -= anchor[0];
        decoded[3] -= anchor[1];
        decoded[0] -= decoded[3] * 0.5f;
        decoded[1] -= decoded[3] * 0.5f;

        if (decoded[2] < 0.0f || decoded[3] < 0.0f) continue;

        PalmDetection detection;
        detection.score = score;
        detection.box = {decoded[0], decoded[1], decoded[2], decoded[3]};
        std::copy_n(decoded.data() + 4, detection.keypoints.size(), detection.keypoints.begin());
        detections.push_back(detection);
    }

    return detections;
}

}