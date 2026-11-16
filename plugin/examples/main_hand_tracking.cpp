// Unit tests for the pure hand tracking algorithms (no camera needed).
#include "../include/spectacularAI/unity/hand_tracking.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "CHECK failed: " #condition " (line " << __LINE__ << ")" << std::endl; \
            return 1; \
        } \
    } while (0)

int main() {
    using namespace saiHandTracking;

    // --- anchors and palm decoding ---
    const auto& anchors = palmAnchors();
    CHECK(anchors.size() == 896);

    std::vector<float> scores(anchors.size(), -10.0f);
    std::vector<float> regressors(anchors.size() * 18, 0.0f);
    scores[0] = 10.0f;
    regressors[2] = 32.0f;
    regressors[3] = 32.0f;
    const auto detections = decodePalmDetections(scores, regressors, 0.5f, true);
    CHECK(detections.size() == 1);
    CHECK(detections[0].pdScore > 0.99f);
    CHECK(std::fabs(detections[0].pdBox[2] - 0.25f) < 0.0001f);
    CHECK(std::fabs(detections[0].pdBox[3] - 0.25f) < 0.0001f);

    // --- NMS keeps the best of two overlapping boxes ---
    HandRegion a, b, c;
    a.pdScore = 0.9f; a.pdBox = {{0.10f, 0.10f, 0.20f, 0.20f}};
    b.pdScore = 0.8f; b.pdBox = {{0.11f, 0.11f, 0.20f, 0.20f}};
    c.pdScore = 0.7f; c.pdBox = {{0.60f, 0.60f, 0.20f, 0.20f}};
    const auto kept = nonMaxSuppression({b, c, a}, 0.3f, 2);
    CHECK(kept.size() == 2);
    CHECK(kept[0].pdScore == 0.9f && kept[1].pdScore == 0.7f);

    // --- metric placement: project a known 3D hand, check that the translation is recovered ---
    Intrinsics k;
    k.fx = 386.8f; k.fy = 386.8f; k.cx = 317.1f; k.cy = 181.8f;
    const float rotation = 0.4f;
    const float trueT[3] = {0.07f, -0.03f, 0.55f};
    std::array<float, 63> worldRoi{};
    std::array<float, 42> pixels{};
    for (int i = 0; i < kNumLandmarks; ++i) {
        // some hand like cloud in the ROI frame
        const float x = 0.09f * std::sin(i * 0.9f);
        const float y = 0.10f * std::cos(i * 0.7f) - 0.03f;
        const float z = 0.03f * std::sin(i * 1.7f);
        worldRoi[i * 3] = x; worldRoi[i * 3 + 1] = y; worldRoi[i * 3 + 2] = z;
        const float rx = x * std::cos(rotation) - y * std::sin(rotation);
        const float ry = x * std::sin(rotation) + y * std::cos(rotation);
        const float X = rx + trueT[0], Y = ry + trueT[1], Z = z + trueT[2];
        pixels[i * 2] = k.fx * X / Z + k.cx;
        pixels[i * 2 + 1] = k.fy * Y / Z + k.cy;
    }
    const HandPlacement placement = placeHand(worldRoi, rotation, pixels, k);
    CHECK(placement.ok);
    CHECK(std::fabs(placement.translation[0] - trueT[0]) < 1e-3f);
    CHECK(std::fabs(placement.translation[1] - trueT[1]) < 1e-3f);
    CHECK(std::fabs(placement.translation[2] - trueT[2]) < 1e-3f);

    // a hand twice as big as the model assumes looks twice as close/big: scaling the model by the scale factor
    // must scale the recovered translation by the same factor
    std::array<float, 63> bigWorld = worldRoi;
    for (float& v : bigWorld) v *= 1.2f;
    const HandPlacement big = placeHand(bigWorld, rotation, pixels, k);
    CHECK(big.ok);
    CHECK(std::fabs(big.translation[2] / placement.translation[2] - 1.2f) < 1e-3f);

    // --- ROI from landmarks of a vertical hand: no rotation expected ---
    HandRegion hand;
    for (int i = 0; i < kNumLandmarks; ++i) {
        hand.landmarks[i * 2] = 300.0f + (i % 5) * 6.0f;
        hand.landmarks[i * 2 + 1] = 400.0f - i * 8.0f;
    }
    hand.landmarks[0] = 320.0f; hand.landmarks[1] = 420.0f;        // wrist
    hand.landmarks[5 * 2] = 305.0f; hand.landmarks[5 * 2 + 1] = 340.0f;
    hand.landmarks[9 * 2] = 320.0f; hand.landmarks[9 * 2 + 1] = 335.0f;
    hand.landmarks[13 * 2] = 335.0f; hand.landmarks[13 * 2 + 1] = 340.0f;
    const HandRegion next = roiFromLandmarks(hand);
    CHECK(std::fabs(next.rotation) < 0.1f);
    CHECK(next.rectSize > 50.0f);

    std::cout << "Hand tracking unit tests passed" << std::endl;
    return 0;
}
