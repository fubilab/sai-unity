#pragma once

// Hand tracking algorithms (MediaPipe palm detection + hand landmark models).
//
// This file only contains pure algorithms with no DepthAI dependency so that they can be unit tested:
// anchor generation, palm detection decoding, ROI geometry, image warping, landmark decoding,
// gesture recognition and metric 3D placement of a hand in the camera frame.
//
// It is a port of the host-mode logic of https://github.com/geaxgx/depthai_hand_tracker
//
// Coordinate conventions
//  * "square image": the camera image padded (zeros) to a square of side S = max(width, height).
//    All ROI geometry is expressed in square-image pixels, like the reference implementation.
//  * "camera frame": x right, y down, z forward (OpenCV convention), meters.

#include <array>
#include <cstdint>
#include <vector>

namespace saiHandTracking {

constexpr int kNumLandmarks = 21;
constexpr int kPalmInputSize = 128;
constexpr int kLandmarkInputSize = 224;

using RectPoints = std::array<std::array<float, 2>, 4>;

/** A hand being tracked: palm detection output, rotated ROI, and landmark inference results. */
struct HandRegion {
    // Palm detection (normalized in the square image)
    float pdScore = 0.0f;
    std::array<float, 4> pdBox{};          // x, y, w, h
    std::array<float, 14> pdKeypoints{};   // 7 x (x, y)

    // Rotated ROI in square-image pixels
    float rectXCenter = 0.0f;
    float rectYCenter = 0.0f;
    float rectSize = 0.0f;                 // width == height
    float rotation = 0.0f;                 // radians
    RectPoints rectPoints{};               // points go clockwise, the first one is bottom-left

    // Landmark model outputs
    float lmScore = 0.0f;
    float handedness = 0.5f;               // > 0.5 means right hand
    std::array<float, 63> normLandmarks{}; // normalized (0..1) inside the ROI, z relative
    std::array<float, 42> landmarks{};     // x, y in square-image pixels
    std::array<float, 63> worldLandmarks{};// meters, in ROI frame (hand upright), hand centered
    int gesture = -1;
    bool hasLandmarks = false;
};

struct Anchor {
    float x, y, w, h;
};

const std::vector<Anchor>& palmAnchors();

/** Decode raw palm detection outputs (896 scores, 896 x 18 regressors). */
std::vector<HandRegion> decodePalmDetections(
    const std::vector<float>& scores,
    const std::vector<float>& regressors,
    float scoreThreshold,
    bool bestOnly);

std::vector<HandRegion> nonMaxSuppression(
    std::vector<HandRegion> regions,
    float iouThreshold,
    std::size_t maxCount);

/** Palm detection -> rotated ROI around the whole hand (square image side = squareSize). */
void palmToRoi(HandRegion& region, float squareSize);

/** Landmarks of this hand -> ROI for the next frame. Requires landmarks in square-image pixels. */
HandRegion roiFromLandmarks(const HandRegion& hand);

/** Resize the camera image (planar RGB, 3 planes) letterboxed into a planar BGR input for the palm model. */
void makePalmInput(
    const std::uint8_t* planarRgb, int width, int height, std::vector<std::uint8_t>& planarBgr128);

/** Warp the rotated ROI of a hand out of the camera image into a planar BGR 224x224 landmark model input. */
void makeLandmarkInput(
    const std::uint8_t* planarRgb, int width, int height, const RectPoints& rect, std::vector<std::uint8_t>& planarBgr224);

/** Decode the landmark model outputs into the region (score thresholding is up to the caller). */
void decodeLandmarks(
    HandRegion& hand,
    const std::vector<float>& score,
    const std::vector<float>& handedness,
    const std::vector<float>& landmarks,
    const std::vector<float>& worldLandmarks,
    float scoreThreshold);

/** 0 = none, 1..5 = number of fingers (FIVE = 5), 6 = FIST, 7 = OK, 8 = PEACE */
void recognizeGesture(HandRegion& hand);

// ---- Metric 3D placement ----

struct Intrinsics {
    float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
    bool valid() const { return fx > 0.0f && fy > 0.0f; }
};

/** Hand landmarks (hand-centered, image-aligned axes) fitted to the observed 2D landmarks. */
struct HandPlacement {
    std::array<float, 3> translation{};      // camera-frame position of the hand-centered origin when scale == 1
    std::array<float, 63> unitScaleCamera{}; // camera-frame landmarks computed with scale == 1
    bool ok = false;
};

/**
 * World landmarks of the model are rotated to the image axes using the ROI rotation. The translation is
 * obtained in closed form by least squares so that the metric landmarks reproject on the 2D landmarks.
 * @param landmarksImagePx 21 x (x, y) in image pixels (not square-image pixels)
 */
HandPlacement placeHand(
    const std::array<float, 63>& worldLandmarksRoi,
    float roiRotation,
    const std::array<float, 42>& landmarksImagePx,
    const Intrinsics& intrinsics);

/** Number of floats per hand in the flat output buffer, see sai_depthai_session_get_hand_frame */
constexpr int kHandFloatStride = 8 + 63 * 3;

}
