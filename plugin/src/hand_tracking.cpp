#include "../include/spectacularAI/unity/hand_tracking.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace saiHandTracking {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float normalizeRadians(float angle) {
    return angle - 2.0f * kPi * std::floor((angle + kPi) / (2.0f * kPi));
}

float distance3(const float* a, const float* b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float angle3(const float* a, const float* b, const float* c) {
    const float ba[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    const float bc[3] = {c[0] - b[0], c[1] - b[1], c[2] - b[2]};
    const float baLength = distance3(a, b);
    const float bcLength = distance3(c, b);
    if (baLength <= std::numeric_limits<float>::epsilon() ||
        bcLength <= std::numeric_limits<float>::epsilon()) {
        return 0.0f;
    }
    float cosine = (ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]) / (baLength * bcLength);
    cosine = std::max(-1.0f, std::min(1.0f, cosine));
    return std::acos(cosine) * 180.0f / kPi;
}

// The reference implementation (and MediaPipe) converts the ROI corners to integers. Do the same so that
// the model input is computed exactly like in the reference.
RectPoints rotatedRectToPoints(float cx, float cy, float w, float h, float rotation) {
    const float b = std::cos(rotation) * 0.5f;
    const float a = std::sin(rotation) * 0.5f;
    const float p0x = cx - a * h - b * w;
    const float p0y = cy + b * h - a * w;
    const float p1x = cx + a * h - b * w;
    const float p1y = cy - b * h - a * w;
    const float p2x = static_cast<float>(static_cast<int>(2.0f * cx - p0x));
    const float p2y = static_cast<float>(static_cast<int>(2.0f * cy - p0y));
    const float p3x = static_cast<float>(static_cast<int>(2.0f * cx - p1x));
    const float p3y = static_cast<float>(static_cast<int>(2.0f * cy - p1y));
    RectPoints points;
    points[0] = {{static_cast<float>(static_cast<int>(p0x)), static_cast<float>(static_cast<int>(p0y))}};
    points[1] = {{static_cast<float>(static_cast<int>(p1x)), static_cast<float>(static_cast<int>(p1y))}};
    points[2] = {{p2x, p2y}};
    points[3] = {{p3x, p3y}};
    return points;
}

// Bilinear sample with zeros outside of the image (same as warpAffine on a zero padded image).
inline float sampleBilinear(const std::uint8_t* plane, int width, int height, float x, float y) {
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const float fx = x - static_cast<float>(x0);
    const float fy = y - static_cast<float>(y0);
    const auto pixel = [&](int px, int py) -> float {
        if (px < 0 || py < 0 || px >= width || py >= height) return 0.0f;
        return static_cast<float>(plane[static_cast<std::size_t>(py) * width + px]);
    };
    const float top = pixel(x0, y0) * (1.0f - fx) + pixel(x0 + 1, y0) * fx;
    const float bottom = pixel(x0, y0 + 1) * (1.0f - fx) + pixel(x0 + 1, y0 + 1) * fx;
    return top * (1.0f - fy) + bottom * fy;
}

inline std::uint8_t toByte(float value) {
    return static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, value + 0.5f)));
}

}

const std::vector<Anchor>& palmAnchors() {
    static const std::vector<Anchor> anchors = []() {
        // https://github.com/google/mediapipe/blob/master/mediapipe/modules/palm_detection/palm_detection_cpu.pbtxt
        constexpr float minScale = 0.1484375f;
        constexpr float maxScale = 0.75f;
        const int strides[4] = {8, 16, 16, 16};
        const int inputWidth = kPalmInputSize;
        const int inputHeight = kPalmInputSize;
        std::vector<Anchor> result;
        int layerIndex = 0;
        while (layerIndex < 4) {
            int lastSameStrideLayer = layerIndex;
            int anchorsPerCell = 0;
            while (lastSameStrideLayer < 4 && strides[lastSameStrideLayer] == strides[layerIndex]) {
                // aspect ratio 1.0 plus one interpolated scale anchor
                anchorsPerCell += 2;
                ++lastSameStrideLayer;
            }
            const int featureMapHeight = (inputHeight + strides[layerIndex] - 1) / strides[layerIndex];
            const int featureMapWidth = (inputWidth + strides[layerIndex] - 1) / strides[layerIndex];
            for (int y = 0; y < featureMapHeight; ++y) {
                for (int x = 0; x < featureMapWidth; ++x) {
                    for (int anchorIndex = 0; anchorIndex < anchorsPerCell; ++anchorIndex) {
                        result.push_back({(x + 0.5f) / featureMapWidth, (y + 0.5f) / featureMapHeight, 1.0f, 1.0f});
                    }
                }
            }
            layerIndex = lastSameStrideLayer;
        }
        return result;
    }();
    return anchors;
}

std::vector<HandRegion> decodePalmDetections(
        const std::vector<float>& scores,
        const std::vector<float>& regressors,
        float scoreThreshold,
        bool bestOnly) {
    constexpr int valuesPerDetection = 18;
    constexpr float scale = static_cast<float>(kPalmInputSize);
    const std::vector<Anchor>& anchors = palmAnchors();
    const std::size_t count = std::min(anchors.size(), std::min(scores.size(), regressors.size() / valuesPerDetection));

    std::size_t bestIndex = count;
    if (bestOnly) {
        float bestScore = -std::numeric_limits<float>::infinity();
        for (std::size_t index = 0; index < count; ++index) {
            if (scores[index] > bestScore) {
                bestScore = scores[index];
                bestIndex = index;
            }
        }
        if (bestIndex == count || 1.0f / (1.0f + std::exp(-bestScore)) < scoreThreshold) return {};
    }

    std::vector<HandRegion> regions;
    for (std::size_t index = 0; index < count; ++index) {
        if (bestOnly && index != bestIndex) continue;
        const float score = 1.0f / (1.0f + std::exp(-scores[index]));
        if (!bestOnly && score <= scoreThreshold) continue;

        const Anchor& anchor = anchors[index];
        const float* regression = regressors.data() + index * valuesPerDetection;
        float decoded[valuesPerDetection];
        for (int valueIndex = 0; valueIndex < valuesPerDetection; ++valueIndex) {
            const float anchorCoordinate = valueIndex % 2 == 0 ? anchor.x : anchor.y;
            const float anchorSize = valueIndex % 2 == 0 ? anchor.w : anchor.h;
            decoded[valueIndex] = regression[valueIndex] * anchorSize / scale + anchorCoordinate;
        }
        // width and height must not contain the anchor center
        decoded[2] -= anchor.x;
        decoded[3] -= anchor.y;
        // center -> top left
        decoded[0] -= decoded[3] * 0.5f;
        decoded[1] -= decoded[3] * 0.5f;
        if (decoded[2] < 0.0f || decoded[3] < 0.0f) continue;

        HandRegion region;
        region.pdScore = score;
        region.pdBox = {{decoded[0], decoded[1], decoded[2], decoded[3]}};
        for (int keypoint = 0; keypoint < 14; ++keypoint) region.pdKeypoints[keypoint] = decoded[4 + keypoint];
        regions.push_back(region);
    }
    return regions;
}

std::vector<HandRegion> nonMaxSuppression(
        std::vector<HandRegion> regions,
        float iouThreshold,
        std::size_t maxCount) {
    std::stable_sort(regions.begin(), regions.end(), [](const HandRegion& left, const HandRegion& right) {
        return left.pdScore > right.pdScore;
    });
    // Boxes are quantized like in the reference implementation (cv2.dnn.NMSBoxes needs integer boxes)
    const auto quantize = [](float value) { return static_cast<int>(value * 1000.0f); };
    std::vector<HandRegion> selected;
    for (const HandRegion& candidate : regions) {
        bool overlaps = false;
        for (const HandRegion& existing : selected) {
            const int cl = quantize(candidate.pdBox[0]), ct = quantize(candidate.pdBox[1]);
            const int cw = quantize(candidate.pdBox[2]), ch = quantize(candidate.pdBox[3]);
            const int el = quantize(existing.pdBox[0]), et = quantize(existing.pdBox[1]);
            const int ew = quantize(existing.pdBox[2]), eh = quantize(existing.pdBox[3]);
            const int left = std::max(cl, el);
            const int top = std::max(ct, et);
            const int right = std::min(cl + cw, el + ew);
            const int bottom = std::min(ct + ch, et + eh);
            const float intersection = static_cast<float>(std::max(0, right - left)) *
                static_cast<float>(std::max(0, bottom - top));
            const float unionArea = static_cast<float>(cw * ch + ew * eh) - intersection;
            if (unionArea > 0.0f && intersection / unionArea > iouThreshold) {
                overlaps = true;
                break;
            }
        }
        if (!overlaps) {
            selected.push_back(candidate);
            if (selected.size() >= maxCount) break;
        }
    }
    return selected;
}

void palmToRoi(HandRegion& region, float squareSize, float cropX, float cropY) {
    // palm_detection_detection_to_roi.pbtxt: rotation from wrist center to middle finger MCP
    const float targetAngle = kPi * 0.5f;
    const float rectW = region.pdBox[2];
    const float rectH = region.pdBox[3];
    const float xCenter = region.pdBox[0] + rectW * 0.5f;
    const float yCenter = region.pdBox[1] + rectH * 0.5f;
    const float x0 = region.pdKeypoints[0], y0 = region.pdKeypoints[1];
    const float x1 = region.pdKeypoints[4], y1 = region.pdKeypoints[5];
    region.rotation = normalizeRadians(targetAngle - std::atan2(-(y1 - y0), x1 - x0));

    // Expand and shift the rectangle so that it covers the whole hand. The reference uses 2.9 instead of
    // MediaPipe's 2.6, otherwise finger tips can end up outside the ROI.
    const float scaleXY = 2.9f;
    const float shiftY = -0.5f;
    const float xShift = -squareSize * rectH * shiftY * std::sin(region.rotation);
    const float yShift = squareSize * rectH * shiftY * std::cos(region.rotation);
    region.rectXCenter = xCenter * squareSize + xShift + cropX;
    region.rectYCenter = yCenter * squareSize + yShift + cropY;
    const float longSide = std::max(rectW * squareSize, rectH * squareSize);
    region.rectSize = longSide * scaleXY;
    region.rectPoints = rotatedRectToPoints(
        region.rectXCenter, region.rectYCenter, region.rectSize, region.rectSize, region.rotation);
}

HandRegion roiFromLandmarks(const HandRegion& hand) {
    const float* lm = hand.landmarks.data();
    const auto at = [lm](int index, int axis) { return lm[index * 2 + axis]; };

    const float x0 = at(0, 0), y0 = at(0, 1);
    const float x1 = 0.25f * (at(5, 0) + at(13, 0)) + 0.5f * at(9, 0);
    const float y1 = 0.25f * (at(5, 1) + at(13, 1)) + 0.5f * at(9, 1);
    const float rotation = normalizeRadians(0.5f * kPi - std::atan2(y0 - y1, x1 - x0));

    static const int ids[12] = {0, 1, 2, 3, 5, 6, 9, 10, 13, 14, 17, 18};
    float minX = std::numeric_limits<float>::max(), minY = minX;
    float maxX = -minX, maxY = -minX;
    for (int id : ids) {
        minX = std::min(minX, at(id, 0));
        maxX = std::max(maxX, at(id, 0));
        minY = std::min(minY, at(id, 1));
        maxY = std::max(maxY, at(id, 1));
    }
    const float centerX = 0.5f * (minX + maxX);
    const float centerY = 0.5f * (minY + maxY);

    const float c = std::cos(rotation);
    const float s = std::sin(rotation);
    float projMinX = std::numeric_limits<float>::max(), projMinY = projMinX;
    float projMaxX = -projMinX, projMaxY = -projMinX;
    for (int id : ids) {
        const float ox = at(id, 0) - centerX;
        const float oy = at(id, 1) - centerY;
        const float px = ox * c + oy * s;
        const float py = -ox * s + oy * c;
        projMinX = std::min(projMinX, px);
        projMaxX = std::max(projMaxX, px);
        projMinY = std::min(projMinY, py);
        projMaxY = std::max(projMaxY, py);
    }
    const float projCenterX = 0.5f * (projMinX + projMaxX);
    const float projCenterY = 0.5f * (projMinY + projMaxY);
    const float width = projMaxX - projMinX;
    const float height = projMaxY - projMinY;
    const float roiCenterX = c * projCenterX - s * projCenterY + centerX;
    const float roiCenterY = s * projCenterX + c * projCenterY + centerY;

    HandRegion next;
    next.rectSize = 2.0f * std::max(width, height);
    next.rectXCenter = roiCenterX + 0.1f * height * s;
    next.rectYCenter = roiCenterY - 0.1f * height * c;
    next.rotation = rotation;
    next.rectPoints = rotatedRectToPoints(
        next.rectXCenter, next.rectYCenter, next.rectSize, next.rectSize, next.rotation);
    return next;
}

void makePalmInput(
        const std::uint8_t* planarRgb, int width, int height, std::vector<std::uint8_t>& out,
        float cropX, float cropY, float cropSize) {
    const int squareSize = std::max(width, height);
    const int padX = (squareSize - width) / 2;
    const int padY = (squareSize - height) / 2;
    const float scale = (cropSize > 0.0f ? cropSize : static_cast<float>(squareSize)) / kPalmInputSize;
    const std::size_t planeSize = static_cast<std::size_t>(width) * height;
    out.assign(static_cast<std::size_t>(kPalmInputSize) * kPalmInputSize * 3, 0);
    for (int y = 0; y < kPalmInputSize; ++y) {
        const float sy = (y + 0.5f) * scale - 0.5f + cropY - padY;
        for (int x = 0; x < kPalmInputSize; ++x) {
            const float sx = (x + 0.5f) * scale - 0.5f + cropX - padX;
            const std::size_t outIndex = static_cast<std::size_t>(y) * kPalmInputSize + x;
            for (int channel = 0; channel < 3; ++channel) {
                // output is BGR, input is RGB
                const std::uint8_t* plane = planarRgb + planeSize * (2 - channel);
                out[channel * kPalmInputSize * kPalmInputSize + outIndex] =
                    toByte(sampleBilinear(plane, width, height, sx, sy));
            }
        }
    }
}

void makeLandmarkInput(
        const std::uint8_t* planarRgb, int width, int height, const RectPoints& rect, std::vector<std::uint8_t>& out) {
    const int squareSize = std::max(width, height);
    const float padX = static_cast<float>((squareSize - width) / 2);
    const float padY = static_cast<float>((squareSize - height) / 2);
    const std::size_t planeSize = static_cast<std::size_t>(width) * height;
    constexpr int size = kLandmarkInputSize;
    out.assign(static_cast<std::size_t>(size) * size * 3, 0);

    const float originX = rect[1][0], originY = rect[1][1];
    const float ux = (rect[2][0] - rect[1][0]) / size, uy = (rect[2][1] - rect[1][1]) / size;
    const float vx = (rect[3][0] - rect[2][0]) / size, vy = (rect[3][1] - rect[2][1]) / size;
    const std::uint8_t* planes[3] = {planarRgb + planeSize * 2, planarRgb + planeSize, planarRgb};
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float sx = originX + x * ux + y * vx - padX;
            const float sy = originY + x * uy + y * vy - padY;
            const std::size_t outIndex = static_cast<std::size_t>(y) * size + x;
            for (int channel = 0; channel < 3; ++channel) {
                out[static_cast<std::size_t>(channel) * size * size + outIndex] =
                    toByte(sampleBilinear(planes[channel], width, height, sx, sy));
            }
        }
    }
}

void recognizeGesture(HandRegion& hand) {
    const float* landmarks = hand.normLandmarks.data();
    const float thumbAngle = angle3(landmarks + 0, landmarks + 3, landmarks + 6) +
        angle3(landmarks + 3, landmarks + 6, landmarks + 9) +
        angle3(landmarks + 6, landmarks + 9, landmarks + 12);
    const float d35 = distance3(landmarks + 9, landmarks + 15);
    const float d23 = distance3(landmarks + 6, landmarks + 9);
    const int thumb = thumbAngle > 460.0f && d23 > 0.0f && d35 / d23 > 1.2f ? 1 : 0;
    const auto fingerState = [landmarks](int tip, int dip, int pip) {
        if (landmarks[tip * 3 + 1] < landmarks[dip * 3 + 1] && landmarks[dip * 3 + 1] < landmarks[pip * 3 + 1]) return 1;
        if (landmarks[pip * 3 + 1] < landmarks[tip * 3 + 1]) return 0;
        return -1;
    };
    const int index = fingerState(8, 7, 6);
    const int middle = fingerState(12, 11, 10);
    const int ring = fingerState(16, 15, 14);
    const int little = fingerState(20, 19, 18);

    if (thumb == 1 && index == 1 && middle == 1 && ring == 1 && little == 1) hand.gesture = 5;
    else if (thumb == 0 && index == 0 && middle == 0 && ring == 0 && little == 0) hand.gesture = 6;
    else if (thumb == 1 && index == 0 && middle == 0 && ring == 0 && little == 0) hand.gesture = 7;
    else if (thumb == 0 && index == 1 && middle == 1 && ring == 0 && little == 0) hand.gesture = 8;
    else if (thumb == 0 && index == 1 && middle == 0 && ring == 0 && little == 0) hand.gesture = 1;
    else if (thumb == 1 && index == 1 && middle == 0 && ring == 0 && little == 0) hand.gesture = 2;
    else if (thumb == 1 && index == 1 && middle == 1 && ring == 0 && little == 0) hand.gesture = 3;
    else if (thumb == 0 && index == 1 && middle == 1 && ring == 1 && little == 1) hand.gesture = 4;
    else hand.gesture = 0;
}

void decodeLandmarks(
        HandRegion& hand,
        const std::vector<float>& score,
        const std::vector<float>& handedness,
        const std::vector<float>& landmarks,
        const std::vector<float>& worldLandmarks,
        float scoreThreshold) {
    hand.hasLandmarks = false;
    hand.lmScore = score.empty() ? 0.0f : score[0];
    if (hand.lmScore <= scoreThreshold || landmarks.size() < 63) return;

    hand.handedness = handedness.empty() ? 0.5f : handedness[0];
    const RectPoints& rect = hand.rectPoints;
    for (int index = 0; index < kNumLandmarks; ++index) {
        const float nx = landmarks[index * 3] / kLandmarkInputSize;
        const float ny = landmarks[index * 3 + 1] / kLandmarkInputSize;
        const float nz = landmarks[index * 3 + 2] / kLandmarkInputSize;
        hand.normLandmarks[index * 3] = nx;
        hand.normLandmarks[index * 3 + 1] = ny;
        hand.normLandmarks[index * 3 + 2] = nz;
        // affine map (0,0)->p1, (1,0)->p2, (1,1)->p3
        hand.landmarks[index * 2] = rect[1][0] + nx * (rect[2][0] - rect[1][0]) + ny * (rect[3][0] - rect[2][0]);
        hand.landmarks[index * 2 + 1] = rect[1][1] + nx * (rect[2][1] - rect[1][1]) + ny * (rect[3][1] - rect[2][1]);
    }
    if (worldLandmarks.size() >= 63) {
        std::copy(worldLandmarks.begin(), worldLandmarks.begin() + 63, hand.worldLandmarks.begin());
    }
    hand.hasLandmarks = true;
    recognizeGesture(hand);
}

HandPlacement placeHand(
        const std::array<float, 63>& worldLandmarksRoi,
        float roiRotation,
        const std::array<float, 42>& landmarksImagePx,
        const Intrinsics& intrinsics) {
    HandPlacement result;
    if (!intrinsics.valid()) return result;

    // Rotate the world landmarks from the ROI frame (hand upright) to the image axes
    const float c = std::cos(roiRotation);
    const float s = std::sin(roiRotation);
    std::array<float, 63> world;
    for (int index = 0; index < kNumLandmarks; ++index) {
        const float x = worldLandmarksRoi[index * 3];
        const float y = worldLandmarksRoi[index * 3 + 1];
        world[index * 3] = x * c - y * s;
        world[index * 3 + 1] = x * s + y * c;
        world[index * 3 + 2] = worldLandmarksRoi[index * 3 + 2];
    }

    // Solve for translation T: (u - cx) (Pz + Tz) = fx (Px + Tx) and (v - cy) (Pz + Tz) = fy (Py + Ty)
    // This is linear in T, solved in closed form via normal equations.
    double ata[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    double atb[3] = {0, 0, 0};
    const auto addRow = [&](double a0, double a1, double a2, double b) {
        const double row[3] = {a0, a1, a2};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) ata[i][j] += row[i] * row[j];
            atb[i] += row[i] * b;
        }
    };
    for (int index = 0; index < kNumLandmarks; ++index) {
        const double du = landmarksImagePx[index * 2] - intrinsics.cx;
        const double dv = landmarksImagePx[index * 2 + 1] - intrinsics.cy;
        const double px = world[index * 3], py = world[index * 3 + 1], pz = world[index * 3 + 2];
        addRow(intrinsics.fx, 0.0, -du, du * pz - intrinsics.fx * px);
        addRow(0.0, intrinsics.fy, -dv, dv * pz - intrinsics.fy * py);
    }

    // Gaussian elimination with partial pivoting
    double m[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) m[i][j] = ata[i][j];
        m[i][3] = atb[i];
    }
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::fabs(m[row][col]) > std::fabs(m[pivot][col])) pivot = row;
        }
        if (std::fabs(m[pivot][col]) < 1e-9) return result;
        if (pivot != col) {
            for (int k = 0; k < 4; ++k) std::swap(m[pivot][k], m[col][k]);
        }
        for (int row = col + 1; row < 3; ++row) {
            const double factor = m[row][col] / m[col][col];
            for (int k = col; k < 4; ++k) m[row][k] -= factor * m[col][k];
        }
    }
    double t[3];
    for (int row = 2; row >= 0; --row) {
        double value = m[row][3];
        for (int k = row + 1; k < 3; ++k) value -= m[row][k] * t[k];
        t[row] = value / m[row][row];
    }
    if (!(t[2] > 0.03) || !(t[2] < 5.0)) return result;

    result.translation = {{static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2])}};
    for (int index = 0; index < kNumLandmarks; ++index) {
        result.unitScaleCamera[index * 3] = world[index * 3] + result.translation[0];
        result.unitScaleCamera[index * 3 + 1] = world[index * 3 + 1] + result.translation[1];
        result.unitScaleCamera[index * 3 + 2] = world[index * 3 + 2] + result.translation[2];
    }
    result.ok = true;
    return result;
}

}
