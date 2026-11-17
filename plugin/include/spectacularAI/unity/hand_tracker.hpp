#pragma once

// Threaded hand tracker running next to the Spectacular AI VIO session.
//
// The OAK-D device is owned by the Spectacular AI pipeline. The hand tracking neural networks (palm detection
// and hand landmarks) are extra nodes of the same DepthAI pipeline. This class feeds them from a worker thread
// and decodes the results, so that the Unity main thread never blocks on the device.

#include "hand_tracking.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dai {
class DataInputQueue;
class DataOutputQueue;
class ImgFrame;
}

namespace saiHandTracking {

struct TrackedHand {
    int id = 0;                                  // slot of the hand in this frame, 0 or 1
    float landmarkScore = 0.0f;
    float handedness = 0.5f;                     // averaged, > 0.5 means right hand (see README)
    int gesture = 0;
    float rotation = 0.0f;                       // ROI rotation, radians
    float handScale = 1.0f;                      // user hand size relative to the average hand of the model
    float stereoDepth = 0.0f;                    // measured depth at the palm in meters, 0 = not measured
    bool placed = false;                         // cameraLandmarks are valid
    std::array<float, 63> imageLandmarks{};      // x, y normalized in the image, z relative
    std::array<float, 63> cameraLandmarks{};     // meters, camera frame (x right, y down, z forward)
    std::array<float, 63> worldLandmarks{};      // meters, hand centered, image aligned axes
};

struct HandFrame {
    int64_t sequence = -1;                       // color frame sequence number
    double deviceTimestamp = 0.0;                // seconds, device clock, same clock as the VIO pose times
    double latencyMs = 0.0;                      // frame arrival on host -> result
    double processingMs = 0.0;
    int width = 0;
    int height = 0;
    bool palmDetectionRan = false;
    float palmBestScore = 0.0f;                  // best palm detection score of this frame (0..1), when palmDetectionRan
    int skippedCandidates = 0;                   // diagnostics: palm candidates not run through the landmark model, see rejectionMemoryMs
    int palmCrop = -1;                           // diagnostics: crop used by the palm detection (0 center, 1 left, 2 right, 3 whole frame)
    float landmarkBestScore = 0.0f;              // best landmark confidence of this frame, also of rejected hands (diagnostics)
    std::array<float, 4> firstRoi{};             // diagnostics: ROI of the first hand candidate: center x, y, size (square-image pixels), rotation (deg)
    std::vector<TrackedHand> hands;
};

struct Settings {
    float palmScoreThreshold = 0.5f;
    float palmNmsThreshold = 0.3f;
    float landmarkScoreThreshold = 0.5f;
    int maxHands = 2;
    bool solo = false;
    // Duo mode: palm detection is run every N frames when only one hand is tracked, to find the second hand
    int singleHandTolerance = 10;
    bool handednessAverage = true;

    // Offline testing: feed frames recorded by the example (color.raw) instead of the camera
    std::string replayPath;
    bool replayRealtime = false;
    // Restart the replay from the beginning at the end of the file (stress tests)
    bool replayLoop = false;
    // Optional JSON lines log with every result
    std::string logPath;

    // Power and heat. The networks run on a device that is also busy with the VIO, which gets hot.
    // While no hand is tracked, palm detection is run at most every searchIntervalMs.
    int searchIntervalMs = 100;
    // While searching, palm detection looks at one square crop of the frame per run, cycling through the center, the
    // left, the right and the whole frame (letterboxed). A hand is far easier to find when it fills more of the 128 x
    // 128 pixel input of the palm model: in the whole 16:9 frame a hand at arm's length is only ~30 pixels there.
    bool searchCrops = true;
    // A palm candidate that the landmark model rejected is not run through it again for this long (ms) when a
    // candidate shows up at about the same place with a similar palm score: a spot of the scene (a poster, a hand-like
    // shadow) scores above the palm threshold again and again and would cost a landmark run each time. 0 = off.
    int rejectionMemoryMs = 3000;
    // Above this chip temperature (Celsius) the work is reduced in steps, see HandTrackerStatus
    float throttleStartC = 88.0f;
};

struct Endpoints {
    std::shared_ptr<dai::DataOutputQueue> color;      // planar RGB preview frames
    std::shared_ptr<dai::DataOutputQueue> depth;      // optional: stereo depth aligned to the color camera (mm)
    std::shared_ptr<dai::DataInputQueue> palmInput;
    std::shared_ptr<dai::DataOutputQueue> palmOutput;
    std::shared_ptr<dai::DataInputQueue> landmarkInput;
    std::shared_ptr<dai::DataOutputQueue> landmarkOutput;
    // Optional: chip temperature in Celsius
    std::function<float()> chipTemperature;
};

struct HandTrackerStatus {
    float chipTemperature = 0.0f;   // Celsius, 0 = unknown
    // 0 = full speed, 1..3 = increasingly reduced work because the device is hot
    int throttleLevel = 0;
    // True when the neural networks stopped answering (for example an overheated device)
    bool stalled = false;
};

class HandTracker {
public:
    /** onColorFrame is called from the worker thread for every color frame the worker receives */
    HandTracker(
        const Settings& settings,
        const Endpoints& endpoints,
        const Intrinsics& intrinsics,
        std::function<void(std::shared_ptr<dai::ImgFrame>)> onColorFrame);
    ~HandTracker();

    HandTracker(const HandTracker&) = delete;
    HandTracker& operator=(const HandTracker&) = delete;

    std::shared_ptr<const HandFrame> getLatest() const;
    HandTrackerStatus getStatus() const;
    const Intrinsics& getIntrinsics() const { return _intrinsics; }
    bool isReplayFinished() const { return _replayFinished.load(); }

private:
    struct Impl;
    void run();

    const Intrinsics _intrinsics;
    std::unique_ptr<Impl> _impl;
    mutable std::mutex _mutex;
    std::shared_ptr<const HandFrame> _latest;
    HandTrackerStatus _status;
    std::atomic<bool> _stop{false};
    std::atomic<bool> _replayFinished{false};
    std::thread _thread;
};

}
