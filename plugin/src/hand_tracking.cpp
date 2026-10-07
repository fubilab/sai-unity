#include "../include/spectacularAI/unity/hand_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace saiHandTracking {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float calculateScale(float minScale, float maxScale, int strideIndex, int strideCount) {
    if (strideCount == 1) return (minScale + maxScale) / 2.0f;
    return minScale + (maxScale - minScale) * strideIndex / (strideCount - 1);
}

float distance3(const float* a, const float* b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float angle3(const float* a, const float* b, const float* c) {
    const std::array<float, 3> ba = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    const std::array<float, 3> bc = {c[0] - b[0], c[1] - b[1], c[2] - b[2]};
    const float baLength = distance3(a, b);
    const float bcLength = distance3(c, b);
    if (baLength <= std::numeric_limits<float>::epsilon() ||
        bcLength <= std::numeric_limits<float>::epsilon()) {
        return 0.0f;
    }
    const float cosine = std::clamp(
        (ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]) / (baLength * bcLength),
        -1.0f,
        1.0f);
    return std::acos(cosine) * 180.0f / kPi;
}

void recognizeGesture(PalmDetection& detection, const std::array<float, 63>& normalizedLandmarks) {
    const float* landmarks = normalizedLandmarks.data();
    const float thumbAngle = angle3(landmarks + 0, landmarks + 3, landmarks + 6) +
        angle3(landmarks + 3, landmarks + 6, landmarks + 9) +
        angle3(landmarks + 6, landmarks + 9, landmarks + 12);
    const float d35 = distance3(landmarks + 9, landmarks + 15);
    const float d23 = distance3(landmarks + 6, landmarks + 9);
    const int thumb = thumbAngle > 460.0f && d23 > 0.0f && d35 / d23 > 1.2f ? 1 : 0;
    auto fingerState = [landmarks](int tip, int dip, int pip) {
        if (landmarks[tip * 3 + 1] < landmarks[dip * 3 + 1] &&
            landmarks[dip * 3 + 1] < landmarks[pip * 3 + 1]) return 1;
        if (landmarks[pip * 3 + 1] < landmarks[tip * 3 + 1]) return 0;
        return -1;
    };
    const int index = fingerState(8, 7, 6);
    const int middle = fingerState(12, 11, 10);
    const int ring = fingerState(16, 15, 14);
    const int little = fingerState(20, 19, 18);

    if (thumb == 1 && index == 1 && middle == 1 && ring == 1 && little == 1) detection.gesture = 5;
    else if (thumb == 0 && index == 0 && middle == 0 && ring == 0 && little == 0) detection.gesture = 6;
    else if (thumb == 1 && index == 0 && middle == 0 && ring == 0 && little == 0) detection.gesture = 7;
    else if (thumb == 0 && index == 1 && middle == 1 && ring == 0 && little == 0) detection.gesture = 8;
    else if (thumb == 0 && index == 1 && middle == 0 && ring == 0 && little == 0) detection.gesture = 1;
    else if (thumb == 1 && index == 1 && middle == 0 && ring == 0 && little == 0) detection.gesture = 2;
    else if (thumb == 1 && index == 1 && middle == 1 && ring == 0 && little == 0) detection.gesture = 3;
    else if (thumb == 0 && index == 1 && middle == 1 && ring == 1 && little == 1) detection.gesture = 4;
    else detection.gesture = 0;
}

std::array<std::array<float, 2>, 4> calculatePythonRectPoints(
        const PalmDetection& detection,
        float imageWidth,
        float imageHeight) {
    const LandmarkRoi roi = calculateLandmarkRoi(detection, imageWidth, imageHeight);
    const float squareSize = std::max(imageWidth, imageHeight);
    const float padY = (squareSize - imageHeight) * 0.5f;
    const float centerX = roi.centerX * imageWidth;
    const float centerY = roi.centerY * imageHeight + padY;
    const float side = roi.width * imageWidth;
    const float angle = roi.angleDegrees * kPi / 180.0f;
    const float b = std::cos(angle) * 0.5f;
    const float a = std::sin(angle) * 0.5f;
    const float p0x = centerX - a * side - b * side;
    const float p0y = centerY + b * side - a * side;
    const float p1x = centerX + a * side - b * side;
    const float p1y = centerY - b * side - a * side;
    return {{
        {{static_cast<float>(static_cast<int>(p0x)), static_cast<float>(static_cast<int>(p0y))}},
        {{static_cast<float>(static_cast<int>(p1x)), static_cast<float>(static_cast<int>(p1y))}},
        {{static_cast<float>(static_cast<int>(2.0f * centerX - p0x)), static_cast<float>(static_cast<int>(2.0f * centerY - p0y))}},
        {{static_cast<float>(static_cast<int>(2.0f * centerX - p1x)), static_cast<float>(static_cast<int>(2.0f * centerY - p1y))}}}};
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

std::vector<PalmDetection> suppressPalmDetections(
    std::vector<PalmDetection> detections,
    float iouThreshold,
    std::size_t maxDetections) {
    std::sort(detections.begin(), detections.end(), [](const PalmDetection& left, const PalmDetection& right) {
        return left.score > right.score;
    });
    std::vector<PalmDetection> selected;
    for (const PalmDetection& candidate : detections) {
        bool overlaps = false;
        for (const PalmDetection& existing : selected) {
            const int candidateLeft = static_cast<int>(candidate.box[0] * 1000.0f);
            const int candidateTop = static_cast<int>(candidate.box[1] * 1000.0f);
            const int candidateWidth = static_cast<int>(candidate.box[2] * 1000.0f);
            const int candidateHeight = static_cast<int>(candidate.box[3] * 1000.0f);
            const int existingLeft = static_cast<int>(existing.box[0] * 1000.0f);
            const int existingTop = static_cast<int>(existing.box[1] * 1000.0f);
            const int existingWidth = static_cast<int>(existing.box[2] * 1000.0f);
            const int existingHeight = static_cast<int>(existing.box[3] * 1000.0f);
            const int left = std::max(candidateLeft, existingLeft);
            const int top = std::max(candidateTop, existingTop);
            const int right = std::min(candidateLeft + candidateWidth, existingLeft + existingWidth);
            const int bottom = std::min(candidateTop + candidateHeight, existingTop + existingHeight);
            const float intersection = static_cast<float>(
                std::max(0, right - left) * std::max(0, bottom - top));
            const float unionArea = static_cast<float>(candidateWidth * candidateHeight +
                existingWidth * existingHeight) - intersection;
            if (unionArea > 0.0f && intersection / unionArea > iouThreshold) {
                overlaps = true;
                break;
            }
        }
        if (!overlaps) {
            selected.push_back(candidate);
            if (selected.size() >= maxDetections) break;
        }
    }
    return selected;
}

LandmarkRoi calculateLandmarkRoi(
    const PalmDetection& detection,
    float imageWidth,
    float imageHeight) {
    const float x0 = detection.keypoints[0] * imageWidth;
    const float y0 = detection.keypoints[1] * imageHeight;
    const float x1 = detection.keypoints[4] * imageWidth;
    const float y1 = detection.keypoints[5] * imageHeight;
    const float targetAngle = kPi * 0.5f;
    float rotation = targetAngle - std::atan2(-(y1 - y0), x1 - x0);
    rotation = rotation - 2.0f * kPi *
        std::floor((rotation + kPi) / (2.0f * kPi));

    const float palmWidth = detection.box[2] * imageWidth;
    const float palmHeight = detection.box[3] * imageHeight;
    const float longSide = std::max(palmWidth, palmHeight) * 2.9f;
    const float shiftY = -0.5f;
    const float centerX = (detection.box[0] + detection.box[2] * 0.5f) * imageWidth -
        imageHeight * palmHeight * shiftY * std::sin(rotation);
    const float centerY = (detection.box[1] + detection.box[3] * 0.5f) * imageHeight +
        imageHeight * palmHeight * shiftY * std::cos(rotation);

    return {
        centerX / imageWidth,
        centerY / imageHeight,
        longSide / imageWidth,
        longSide / imageHeight,
        rotation * 180.0f / kPi};
}

void decodeLandmark(
    PalmDetection& detection,
    const std::vector<float>& score,
    const std::vector<float>& handedness,
    const std::vector<float>& landmarks,
    const std::vector<float>& worldLandmarks,
    const LandmarkRoi&,
    float imageWidth,
    float imageHeight) {
    constexpr float landmarkInputSize = 224.0f;
    if (score.empty() || landmarks.size() < 63) return;

    detection.landmarkScore = score[0];
    detection.handedness = handedness.empty() ? 0.5f : handedness[0];
    const auto points = calculatePythonRectPoints(detection, imageWidth, imageHeight);
    std::array<float, 63> normalizedLandmarks{};
    for (int index = 0; index < 21; ++index) {
        const float localX = landmarks[index * 3] / landmarkInputSize;
        const float localY = landmarks[index * 3 + 1] / landmarkInputSize;
        const float squareX = points[1][0] + localX * (points[2][0] - points[1][0]) +
            localY * (points[3][0] - points[1][0]);
        const float squareY = points[1][1] + localX * (points[2][1] - points[1][1]) +
            localY * (points[3][1] - points[1][1]);
        const float paddingY = (std::max(imageWidth, imageHeight) - imageHeight) * 0.5f;
        detection.landmarks[index * 3] = squareX / imageWidth;
        detection.landmarks[index * 3 + 1] = (squareY - paddingY) / imageHeight;
        detection.landmarks[index * 3 + 2] = landmarks[index * 3 + 2] / landmarkInputSize;
        normalizedLandmarks[index * 3] = localX;
        normalizedLandmarks[index * 3 + 1] = localY;
        normalizedLandmarks[index * 3 + 2] = detection.landmarks[index * 3 + 2];
    }
    if (worldLandmarks.size() >= 63) {
        std::copy_n(worldLandmarks.data(), 63, detection.worldLandmarks.data());
    }
    detection.hasLandmarks = detection.landmarkScore > 0.5f;
    if (detection.hasLandmarks) recognizeGesture(detection, normalizedLandmarks);
}

}