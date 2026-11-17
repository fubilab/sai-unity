#include "../include/spectacularAI/unity/hand_tracker.hpp"

#include <depthai/depthai.hpp>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace saiHandTracking {
namespace {

using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

float median(std::vector<float>& values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
}

// Frames recorded by the example (SAI_RECORD_DIR), see examples/main_depthai.cpp
class ReplayReader {
public:
    bool open(const std::string& path) {
        _file.close();
        _file.clear();
        _file.open(path, std::ios::binary);
        return _file.is_open();
    }

    // Returns false at the end of the file
    bool next(std::shared_ptr<dai::ImgFrame>& frame, double& recordedHostTime) {
        Header header;
        if (!_file.read(reinterpret_cast<char*>(&header), sizeof(header))) return false;
        if (header.dataSize <= 0 || header.dataSize > 64 * 1024 * 1024) return false;
        std::vector<std::uint8_t> data(static_cast<std::size_t>(header.dataSize));
        if (!_file.read(reinterpret_cast<char*>(data.data()), header.dataSize)) return false;

        frame = std::make_shared<dai::ImgFrame>();
        frame->setType(dai::ImgFrame::Type::RGB888p);
        frame->setSize(static_cast<unsigned int>(header.width), static_cast<unsigned int>(header.height));
        frame->setData(std::move(data));
        frame->setSequenceNum(header.sequence);
        frame->setTimestampDevice(std::chrono::time_point<Clock, Clock::duration>(
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(header.deviceTimestamp))));
        recordedHostTime = header.hostTime;
        return true;
    }

private:
    struct Header {
        int64_t sequence;
        double deviceTimestamp;
        double hostTime;
        int32_t width;
        int32_t height;
        int32_t dataSize;
        int32_t reserved;
    };
    std::ifstream _file;
};

double deviceSeconds(const std::shared_ptr<dai::ImgFrame>& frame) {
    return std::chrono::duration<double>(frame->getTimestampDevice().time_since_epoch()).count();
}

}

struct HandTracker::Impl {
    Impl(const Settings& s, const Endpoints& e, const Intrinsics& k) : settings(s), endpoints(e), intrinsics(k) {}

    Settings settings;
    Endpoints endpoints;
    Intrinsics intrinsics;

    // --- tracking state (reference: HandTracker.next_frame) ---
    bool usePreviousLandmarks = false;
    std::vector<HandRegion> handsFromLandmarks;
    int handsInPreviousFrame = 0;
    int singleHandCount = 0;
    float handednessTotal[2] = {0.0f, 0.0f};
    int handednessCount[2] = {0, 0};

    // --- user hand size, calibrated against stereo depth ---
    float handScale = 1.0f;
    bool handScaleValid = false;
    int handScaleRejected = 0;

    // --- depth ---
    struct DepthFrame {
        double timestamp;
        std::shared_ptr<dai::ImgFrame> frame;
    };
    std::deque<DepthFrame> depthFrames;

    // --- logging / diagnostics ---
    std::ofstream log;
    std::vector<std::uint8_t> palmInputBuffer;
    std::vector<std::uint8_t> landmarkInputBuffer;
    int64_t lastSequence = -1;
    bool warnedFrameType = false;
    std::function<void(std::shared_ptr<dai::ImgFrame>)> onColorFrame;
    int consecutiveTimeouts = 0;
    int searchCropCounter = 0;
    struct RejectedCandidate {
        float x, y, size, score;
        Clock::time_point time;
    };
    std::vector<RejectedCandidate> rejectedCandidates;
    Clock::time_point lastCycleStart = Clock::now() - std::chrono::seconds(10);
    Clock::time_point lastTemperaturePoll = Clock::now() - std::chrono::seconds(10);
    bool debug = false;
    Clock::time_point statsStart = Clock::now();
    int statsFrames = 0;
    double statsLatency = 0.0;
    double statsPalm = 0.0;
    double statsLandmark = 0.0;
    double statsPrep = 0.0;
    double prepMs = 0.0;
    int statsHands = 0;

    std::shared_ptr<HandFrame> process(const std::shared_ptr<dai::ImgFrame>& frame, Clock::time_point captureTime);
    void writeLog(const HandFrame& result, double palmMs, double landmarkMs);

    void pollDepth() {
        if (!endpoints.depth) return;
        while (auto depth = endpoints.depth->tryGet<dai::ImgFrame>()) {
            depthFrames.push_back({deviceSeconds(depth), depth});
            while (depthFrames.size() > 8) depthFrames.pop_front();
        }
    }

    const dai::ImgFrame* nearestDepth(double timestamp, double& deltaSeconds) const {
        const dai::ImgFrame* best = nullptr;
        double bestDelta = 1e9;
        for (const DepthFrame& candidate : depthFrames) {
            const double delta = std::fabs(candidate.timestamp - timestamp);
            if (delta < bestDelta) {
                bestDelta = delta;
                best = candidate.frame.get();
            }
        }
        deltaSeconds = bestDelta;
        return best;
    }

    // Median depth (meters) of a small window, 0 if too few valid pixels
    static float sampleDepth(const dai::ImgFrame& depth, float u, float v, int imageWidth, int imageHeight) {
        const int depthWidth = static_cast<int>(depth.getWidth());
        const int depthHeight = static_cast<int>(depth.getHeight());
        if (depthWidth <= 0 || depthHeight <= 0 ||
            depth.getData().size() < static_cast<std::size_t>(depthWidth) * depthHeight * 2) {
            return 0.0f;
        }
        const std::uint16_t* data = reinterpret_cast<const std::uint16_t*>(depth.getData().data());
        const int cx = static_cast<int>(std::lround(u * depthWidth / imageWidth));
        const int cy = static_cast<int>(std::lround(v * depthHeight / imageHeight));
        constexpr int radius = 3;
        std::vector<float> values;
        values.reserve((2 * radius + 1) * (2 * radius + 1));
        for (int y = cy - radius; y <= cy + radius; ++y) {
            for (int x = cx - radius; x <= cx + radius; ++x) {
                if (x < 0 || y < 0 || x >= depthWidth || y >= depthHeight) continue;
                const std::uint16_t millimeters = data[static_cast<std::size_t>(y) * depthWidth + x];
                if (millimeters >= 150 && millimeters <= 2500) values.push_back(static_cast<float>(millimeters));
            }
        }
        if (values.size() < 8) return 0.0f;
        return median(values) * 0.001f;
    }

    // Fills hand placement: camera landmarks, using stereo depth (when available) to calibrate the hand size
    void place(TrackedHand& out, const HandRegion& hand, int width, int height, const dai::ImgFrame* depth) {
        const int squareSize = std::max(width, height);
        const float padX = static_cast<float>((squareSize - width) / 2);
        const float padY = static_cast<float>((squareSize - height) / 2);
        std::array<float, 42> imagePx;
        for (int i = 0; i < kNumLandmarks; ++i) {
            imagePx[i * 2] = hand.landmarks[i * 2] - padX;
            imagePx[i * 2 + 1] = hand.landmarks[i * 2 + 1] - padY;
        }
        const HandPlacement placement = placeHand(hand.worldLandmarks, hand.rotation, imagePx, intrinsics);
        if (!placement.ok) return;

        // Hand size calibration: compare the depth measured by stereo at palm points with the depth the
        // model predicts for an average hand. The ratio is the hand size factor of this user.
        if (depth != nullptr) {
            static const int palmIds[5] = {0, 5, 9, 13, 17};
            std::vector<float> ratios;
            std::vector<float> depths;
            for (int id : palmIds) {
                const float measured = sampleDepth(*depth, imagePx[id * 2], imagePx[id * 2 + 1], width, height);
                const float predicted = placement.unitScaleCamera[id * 3 + 2];
                if (measured > 0.0f && predicted > 0.03f) {
                    ratios.push_back(measured / predicted);
                    depths.push_back(measured);
                }
            }
            if (ratios.size() >= 3) {
                out.stereoDepth = median(depths);
                const float ratio = median(ratios);
                if (ratio > 0.6f && ratio < 1.6f) {
                    if (!handScaleValid) {
                        handScale = ratio;
                        handScaleValid = true;
                        handScaleRejected = 0;
                    } else if (std::fabs(ratio / handScale - 1.0f) < 0.25f || handScaleRejected > 15) {
                        handScale += 0.05f * (ratio - handScale);
                        handScaleRejected = 0;
                    } else {
                        ++handScaleRejected;
                    }
                }
            }
        }

        out.handScale = handScale;
        for (int i = 0; i < 63; ++i) {
            out.cameraLandmarks[i] = placement.unitScaleCamera[i] * handScale;
        }
        out.placed = true;
    }
};

std::shared_ptr<HandFrame> HandTracker::Impl::process(
        const std::shared_ptr<dai::ImgFrame>& frame, Clock::time_point captureTime) {
    const auto started = Clock::now();
    const int width = static_cast<int>(frame->getWidth());
    const int height = static_cast<int>(frame->getHeight());
    const std::vector<std::uint8_t>& pixels = frame->getData();
    if (width <= 0 || height <= 0 || pixels.size() < static_cast<std::size_t>(width) * height * 3) return nullptr;
    if (frame->getType() != dai::ImgFrame::Type::RGB888p && !warnedFrameType) {
        std::cerr << "[sai-hand] warning: color frames are not planar RGB (type "
            << static_cast<int>(frame->getType()) << "), hand tracking results will be wrong" << std::endl;
        warnedFrameType = true;
    }

    const int squareSize = std::max(width, height);
    const float padX = static_cast<float>((squareSize - width) / 2);
    const float padY = static_cast<float>((squareSize - height) / 2);

    auto result = std::make_shared<HandFrame>();
    result->sequence = frame->getSequenceNum();
    result->deviceTimestamp = deviceSeconds(frame);
    result->width = width;
    result->height = height;

    const bool usedPreviousLandmarks = usePreviousLandmarks;
    std::vector<HandRegion> hands;
    double palmMs = 0.0;
    double landmarkMs = 0.0;

    if (!usedPreviousLandmarks) {
        const auto palmStart = Clock::now();
        while (endpoints.palmOutput->tryGet<dai::NNData>()) {}
        // The part of the (letterboxed) square image the palm model looks at: the whole square, or one square crop
        // of the frame per run, see Settings::searchCrops
        float cropX = 0.0f, cropY = 0.0f, cropSize = static_cast<float>(squareSize);
        if (settings.searchCrops && width != height) {
            const float side = static_cast<float>(std::min(width, height));
            const float spanX = static_cast<float>(width) - side;  // free room along the long side
            const float spanY = static_cast<float>(height) - side;
            const float fractions[3] = {0.5f, 0.0f, 1.0f};         // center, left (top), right (bottom)
            result->palmCrop = searchCropCounter++ % 4;
            if (result->palmCrop < 3) {
                cropX = padX + spanX * fractions[result->palmCrop];
                cropY = padY + spanY * fractions[result->palmCrop];
                cropSize = side;
            }
        }
        makePalmInput(pixels.data(), width, height, palmInputBuffer, cropX, cropY, cropSize);
        auto input = std::make_shared<dai::ImgFrame>();
        input->setType(dai::ImgFrame::Type::BGR888p);
        input->setSize(kPalmInputSize, kPalmInputSize);
        input->setSequenceNum(frame->getSequenceNum());
        input->setData(palmInputBuffer);
        endpoints.palmInput->send(input);

        bool timedOut = false;
        const auto inference = endpoints.palmOutput->get<dai::NNData>(std::chrono::milliseconds(1000), timedOut);
        if (timedOut || !inference) {
            ++consecutiveTimeouts;
            if (consecutiveTimeouts == 1 || consecutiveTimeouts % 30 == 0) {
                std::cerr << "[sai-hand] palm detection timed out (" << consecutiveTimeouts << ")" << std::endl;
            }
            return nullptr;
        }
        consecutiveTimeouts = 0;
        const std::vector<float> scores = inference->getLayerFp16("classificators");
        const std::vector<float> regressors = inference->getLayerFp16("regressors");
        float bestLogit = -std::numeric_limits<float>::infinity();
        for (const float logit : scores) bestLogit = std::max(bestLogit, logit);
        result->palmBestScore = scores.empty() ? 0.0f : 1.0f / (1.0f + std::exp(-bestLogit));
        hands = decodePalmDetections(scores, regressors, settings.palmScoreThreshold, settings.solo);
        if (!settings.solo) {
            hands = nonMaxSuppression(std::move(hands), settings.palmNmsThreshold, static_cast<std::size_t>(settings.maxHands));
        }
        for (HandRegion& hand : hands) palmToRoi(hand, cropSize, cropX, cropY);
        // Do not run the landmark model again on a candidate that it rejected a moment ago at the same place, unless
        // the palm model is clearly more sure this time
        if (settings.rejectionMemoryMs > 0) {
            const auto now = Clock::now();
            const auto expiry = std::chrono::milliseconds(settings.rejectionMemoryMs);
            rejectedCandidates.erase(
                std::remove_if(rejectedCandidates.begin(), rejectedCandidates.end(),
                    [&](const RejectedCandidate& r) { return now - r.time > expiry; }),
                rejectedCandidates.end());
            const std::size_t before = hands.size();
            hands.erase(
                std::remove_if(hands.begin(), hands.end(), [&](const HandRegion& hand) {
                    for (const RejectedCandidate& r : rejectedCandidates) {
                        const float dx = hand.rectXCenter - r.x;
                        const float dy = hand.rectYCenter - r.y;
                        if (std::sqrt(dx * dx + dy * dy) < 0.5f * r.size && hand.pdScore <= r.score + 0.1f) return true;
                    }
                    return false;
                }),
                hands.end());
            result->skippedCandidates = static_cast<int>(before - hands.size());
        }
        result->palmDetectionRan = true;
        // Only one hand is tracked, keep tracking it from its landmarks unless palm detection finds another one
        if (!settings.solo && handsInPreviousFrame == 1 && hands.size() <= 1) hands = handsFromLandmarks;
        palmMs = millisecondsSince(palmStart);
    } else {
        hands = handsFromLandmarks;
    }

    if (!hands.empty()) {
        const auto landmarkStart = Clock::now();
        while (endpoints.landmarkOutput->tryGet<dai::NNData>()) {}
        prepMs = 0.0;
        for (const HandRegion& hand : hands) {
            const auto prepStart = Clock::now();
            makeLandmarkInput(pixels.data(), width, height, hand.rectPoints, landmarkInputBuffer);
            prepMs += millisecondsSince(prepStart);
            auto input = std::make_shared<dai::NNData>();
            input->setLayer("input_1", landmarkInputBuffer);
            endpoints.landmarkInput->send(input);
        }
        for (HandRegion& hand : hands) {
            bool timedOut = false;
            const auto inference = endpoints.landmarkOutput->get<dai::NNData>(std::chrono::milliseconds(1000), timedOut);
            if (timedOut || !inference) {
                ++consecutiveTimeouts;
                if (consecutiveTimeouts == 1 || consecutiveTimeouts % 30 == 0) {
                    std::cerr << "[sai-hand] landmark inference timed out (" << consecutiveTimeouts << ")" << std::endl;
                }
                usePreviousLandmarks = false;
                handsFromLandmarks.clear();
                handsInPreviousFrame = 0;
                return nullptr;
            }
            const auto layer = [&inference](const char* name) {
                return inference->hasLayer(name) ? inference->getLayerFp16(name) : std::vector<float>();
            };
            decodeLandmarks(
                hand,
                layer("Identity_1"),
                layer("Identity_2"),
                layer("Identity_dense/BiasAdd/Add"),
                layer("Identity_3_dense/BiasAdd/Add"),
                settings.landmarkScoreThreshold);
            consecutiveTimeouts = 0;
            result->landmarkBestScore = std::max(result->landmarkBestScore, hand.lmScore);
            if (!hand.hasLandmarks && hand.pdScore > 0.0f && settings.rejectionMemoryMs > 0) {
                // came from palm detection, not from tracking, and is no hand: remember it
                if (rejectedCandidates.size() >= 16) rejectedCandidates.erase(rejectedCandidates.begin());
                rejectedCandidates.push_back({hand.rectXCenter, hand.rectYCenter, hand.rectSize, hand.pdScore, Clock::now()});
            }
        }
        result->firstRoi = {hands[0].rectXCenter, hands[0].rectYCenter, hands[0].rectSize,
                            hands[0].rotation * 57.2957795f};
        landmarkMs = millisecondsSince(landmarkStart);
    }

    hands.erase(
        std::remove_if(hands.begin(), hands.end(), [](const HandRegion& hand) { return !hand.hasLandmarks; }),
        hands.end());

    // Two ROIs may follow the same hand when hands cross: keep the one with the best score
    if (hands.size() == 2) {
        const float dx = hands[0].rectXCenter - hands[1].rectXCenter;
        const float dy = hands[0].rectYCenter - hands[1].rectYCenter;
        if (std::sqrt(dx * dx + dy * dy) < 5.0f) {
            hands = {hands[0].lmScore > hands[1].lmScore ? hands[0] : hands[1]};
        }
    }

    handsFromLandmarks.clear();
    for (const HandRegion& hand : hands) handsFromLandmarks.push_back(roiFromLandmarks(hand));

    int handCount = static_cast<int>(hands.size());
    if (settings.handednessAverage) {
        if (!usedPreviousLandmarks || handsInPreviousFrame != handCount) {
            for (int i = 0; i < 2; ++i) {
                handednessTotal[i] = 0.0f;
                handednessCount[i] = 0;
            }
        }
        for (int i = 0; i < handCount && i < 2; ++i) {
            handednessTotal[i] += hands[i].handedness;
            ++handednessCount[i];
            hands[i].handedness = handednessTotal[i] / handednessCount[i];
        }
    }

    // In duo mode at most one left and one right hand are returned
    if (!settings.solo && handCount == 2 && (hands[0].handedness - 0.5f) * (hands[1].handedness - 0.5f) > 0.0f) {
        hands.resize(1);
        handCount = 1;
    }
    if (!settings.solo) {
        if (handCount == 1) ++singleHandCount;
        else singleHandCount = 0;
    }

    usePreviousLandmarks = true;
    if (handCount == 0) {
        usePreviousLandmarks = false;
    } else if (!settings.solo && handCount == 1 && singleHandCount >= settings.singleHandTolerance) {
        usePreviousLandmarks = false;
        singleHandCount = 0;
    }
    handsInPreviousFrame = handCount;

    // Depth for hand placement
    pollDepth();
    double depthDelta = 0.0;
    const dai::ImgFrame* depth = nearestDepth(result->deviceTimestamp, depthDelta);
    if (depth != nullptr && depthDelta > 0.1) depth = nullptr;

    for (int i = 0; i < handCount; ++i) {
        const HandRegion& hand = hands[i];
        TrackedHand out;
        out.id = i;
        out.landmarkScore = hand.lmScore;
        out.handedness = hand.handedness;
        out.gesture = hand.gesture;
        out.rotation = hand.rotation;
        const float c = std::cos(hand.rotation);
        const float s = std::sin(hand.rotation);
        for (int k = 0; k < kNumLandmarks; ++k) {
            out.imageLandmarks[k * 3] = (hand.landmarks[k * 2] - padX) / width;
            out.imageLandmarks[k * 3 + 1] = (hand.landmarks[k * 2 + 1] - padY) / height;
            out.imageLandmarks[k * 3 + 2] = hand.normLandmarks[k * 3 + 2];
            const float wx = hand.worldLandmarks[k * 3];
            const float wy = hand.worldLandmarks[k * 3 + 1];
            out.worldLandmarks[k * 3] = wx * c - wy * s;
            out.worldLandmarks[k * 3 + 1] = wx * s + wy * c;
            out.worldLandmarks[k * 3 + 2] = hand.worldLandmarks[k * 3 + 2];
        }
        place(out, hand, width, height, depth);
        result->hands.push_back(out);
    }

    result->processingMs = millisecondsSince(started);
    result->latencyMs = millisecondsSince(captureTime);
    writeLog(*result, palmMs, landmarkMs);
    return result;
}

void HandTracker::Impl::writeLog(const HandFrame& result, double palmMs, double landmarkMs) {
    if (debug) {
        ++statsFrames;
        statsLatency += result.latencyMs;
        statsPalm += palmMs;
        statsLandmark += landmarkMs;
        statsPrep += prepMs;
        statsHands += static_cast<int>(result.hands.size());
        const double elapsed = millisecondsSince(statsStart);
        if (elapsed > 2000.0) {
            std::clog << "[sai-hand] " << std::fixed << std::setprecision(1)
                << statsFrames * 1000.0 / elapsed << " fps"
                << " latency " << statsLatency / statsFrames << " ms"
                << " palm " << statsPalm / statsFrames << " ms"
                << " landmarks " << statsLandmark / statsFrames << " ms (crop " << statsPrep / statsFrames << ")"
                << " hands/frame " << static_cast<double>(statsHands) / statsFrames
                << " handScale " << handScale << std::endl;
            statsStart = Clock::now();
            statsFrames = statsHands = 0;
            statsLatency = statsPalm = statsLandmark = statsPrep = 0.0;
        }
    }
    if (!log.is_open()) return;
    log << std::setprecision(6);
    log << "{\"seq\":" << result.sequence << ",\"dev\":" << std::fixed << result.deviceTimestamp << std::defaultfloat
        << ",\"latency_ms\":" << result.latencyMs << ",\"proc_ms\":" << result.processingMs
        << ",\"palm\":" << (result.palmDetectionRan ? 1 : 0) << ",\"skipped\":" << result.skippedCandidates << ",\"palm_crop\":" << result.palmCrop << ",\"palm_best\":" << result.palmBestScore << ",\"lm_best\":" << result.landmarkBestScore
        << ",\"roi\":[" << result.firstRoi[0] << ',' << result.firstRoi[1] << ',' << result.firstRoi[2] << ','
        << result.firstRoi[3] << ']'
        << ",\"palm_ms\":" << palmMs
        << ",\"lm_ms\":" << landmarkMs << ",\"hands\":[";
    for (std::size_t i = 0; i < result.hands.size(); ++i) {
        const TrackedHand& hand = result.hands[i];
        if (i > 0) log << ',';
        log << "{\"lm_score\":" << hand.landmarkScore << ",\"handedness\":" << hand.handedness
            << ",\"gesture\":" << hand.gesture << ",\"stereo_depth\":" << hand.stereoDepth
            << ",\"scale\":" << hand.handScale << ",\"placed\":" << (hand.placed ? 1 : 0) << ",\"landmarks\":[";
        for (int k = 0; k < 63; ++k) log << (k ? "," : "") << hand.imageLandmarks[k];
        log << "],\"cam\":[";
        for (int k = 0; k < 63; ++k) log << (k ? "," : "") << hand.cameraLandmarks[k];
        log << "]}";
    }
    log << "]}" << std::endl;
}

HandTracker::HandTracker(
        const Settings& settings,
        const Endpoints& endpoints,
        const Intrinsics& intrinsics,
        std::function<void(std::shared_ptr<dai::ImgFrame>)> onColorFrame) :
        _intrinsics(intrinsics),
        _impl(new Impl(settings, endpoints, intrinsics)) {
    if (!settings.logPath.empty()) _impl->log.open(settings.logPath);
    _impl->onColorFrame = std::move(onColorFrame);
    _impl->debug = std::getenv("SAI_HAND_TRACKING_DEBUG") != nullptr;
    _thread = std::thread([this]() {
        try {
            run();
        } catch (const std::exception& e) {
            std::cerr << "[sai-hand] tracker thread stopped: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[sai-hand] tracker thread stopped: unknown exception" << std::endl;
        }
        _replayFinished = true;
    });
}

HandTracker::~HandTracker() {
    _stop = true;
    if (_thread.joinable()) _thread.join();
}

std::shared_ptr<const HandFrame> HandTracker::getLatest() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _latest;
}

HandTrackerStatus HandTracker::getStatus() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _status;
}

void HandTracker::run() {
    Impl& impl = *_impl;
    const bool replay = !impl.settings.replayPath.empty();
    ReplayReader reader;
    if (replay && !reader.open(impl.settings.replayPath)) {
        std::cerr << "[sai-hand] cannot open replay file " << impl.settings.replayPath << std::endl;
        return;
    }
    const auto replayStart = Clock::now();
    double firstRecordedTime = -1.0;

    while (!_stop) {
        // Heat: poll the chip temperature and reduce the work when it is high
        if (impl.endpoints.chipTemperature && millisecondsSince(impl.lastTemperaturePoll) > 1000.0) {
            impl.lastTemperaturePoll = Clock::now();
            try {
                const float temperature = impl.endpoints.chipTemperature();
                std::lock_guard<std::mutex> lock(_mutex);
                _status.chipTemperature = temperature;
                const float over = temperature - impl.settings.throttleStartC;
                const int level = temperature <= 0.0f || over < 0.0f ? 0 : (over < 6.0f ? 1 : (over < 10.0f ? 2 : 3));
                if (level != _status.throttleLevel) {
                    std::cerr << "[sai-hand] chip temperature " << temperature << " C, work reduction level "
                        << level << std::endl;
                }
                _status.throttleLevel = level;
            } catch (const std::exception&) {
            }
        }
        int level;
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _status.stalled = impl.consecutiveTimeouts >= 3;
            level = _status.throttleLevel;
        }

        // Pacing. While no hand is tracked only search at a modest rate, when hot also limit the tracking rate.
        static const double kTrackingCycleMs[4] = {0.0, 0.0, 66.0, 150.0};
        static const double kSearchCycleMs[4] = {0.0, 200.0, 300.0, 500.0};
        double minCycleMs = impl.usePreviousLandmarks ? kTrackingCycleMs[level]
            : std::max(static_cast<double>(impl.settings.searchIntervalMs), kSearchCycleMs[level]);
        if (impl.consecutiveTimeouts >= 3) minCycleMs = std::max(minCycleMs, 500.0); // do not hammer a stalled device
        if (!replay && minCycleMs > 0.0) {
            const double since = millisecondsSince(impl.lastCycleStart);
            if (since < minCycleMs) {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(std::min(minCycleMs - since, 10.0)));
                continue;
            }
        }

        std::shared_ptr<dai::ImgFrame> frame;
        Clock::time_point captureTime = Clock::now();
        if (replay) {
            double recordedTime = 0.0;
            if (!reader.next(frame, recordedTime)) {
                if (!impl.settings.replayLoop || !reader.open(impl.settings.replayPath)) break;
                // new loop: reset tracking so that the clip starts from the same state
                impl.usePreviousLandmarks = false;
                impl.handsFromLandmarks.clear();
                impl.handsInPreviousFrame = 0;
                impl.lastSequence = -1;
                firstRecordedTime = -1.0;
                continue;
            }
            if (firstRecordedTime < 0.0) firstRecordedTime = recordedTime;
            if (impl.settings.replayRealtime) {
                const double due = recordedTime - firstRecordedTime;
                const double elapsed = std::chrono::duration<double>(Clock::now() - replayStart).count();
                if (elapsed > due + 0.034) continue; // behind: drop the frame like the live camera would
                if (due > elapsed) std::this_thread::sleep_for(std::chrono::duration<double>(due - elapsed));
            }
            captureTime = Clock::now();
        } else {
            bool timedOut = false;
            frame = impl.endpoints.color->get<dai::ImgFrame>(std::chrono::milliseconds(100), timedOut);
            if (timedOut || !frame) continue;
            while (auto newer = impl.endpoints.color->tryGet<dai::ImgFrame>()) frame = newer;
            captureTime = frame->getTimestamp();
        }
        if (frame->getSequenceNum() == impl.lastSequence) continue;
        impl.lastSequence = frame->getSequenceNum();
        impl.lastCycleStart = Clock::now();
        if (impl.onColorFrame) impl.onColorFrame(frame);

        std::shared_ptr<HandFrame> result = impl.process(frame, captureTime);
        if (result) {
            std::lock_guard<std::mutex> lock(_mutex);
            _latest = result;
        }
    }
    if (replay) _replayFinished = true;
}

}

