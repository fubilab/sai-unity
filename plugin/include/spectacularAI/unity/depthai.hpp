#pragma once

#include <spectacularAI/depthai/plugin.hpp>
#include "types.hpp"
#include "mapping.hpp"
#include "output.hpp"
#include "hand_tracker.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

namespace dai {
class DataOutputQueue;
class DataInputQueue;
class ImgFrame;
}

/**
 * Keep the layout in sync with Configuration.cs in the Unity wrapper. New fields must be appended at the end.
 */
struct ConfigurationWrapper {
    bool useStereo=true;
    bool useSlam=false;
    bool useFeatureTracker=true;
    bool fastVio=false;
    bool useColorStereoCameras=false;
    bool useColor=false;
    const char* mapSavePath="";
    const char* mapLoadPath="";
    const char* aprilTagPath="";
    uint32_t accFrequencyHz=500;
    uint32_t gyroFrequencyHz=400;
    int keyframeCandidateEveryNthFrame=6;
    const char* inputResolution="400p";
    const char* recordingFolder="";
    bool recordingOnly=false;
    bool fastImu=false;
    bool lowLatency=false;
    // Hand tracking. Runs the MediaPipe hand models on the same OAK-D as the VIO.
    bool enableHandTracking=false;
    const char* handTrackingPalmModelPath="";
    const char* handTrackingLandmarkModelPath="";
    // Stereo depth is used to calibrate the user's hand size (metric hand position). It needs the IR dot
    // projector (the right camera of the device only sees IR light), see README.
    bool handTrackingUseDepth=true;
    // 0 = projector off, 1 = max
    float handTrackingIrProjectorIntensity=0.0f;
    int handTrackingMaxHands=2;
    // Frame rate of the color camera when hand tracking is enabled, 0 = unchanged (30). The color camera is a main
    // heat source of the device and the networks do not run faster than ~15 Hz. A lower rate also makes them faster.
    int handTrackingColorFps=0;
};

/** Latest color frame, filled by the hand tracking thread (or polled from the device when it is not running) */
struct ColorFrameQueue {
    void push(const std::shared_ptr<dai::ImgFrame>& frame) {
        std::lock_guard<std::mutex> lock(_mutex);
        _latest = frame;
    }

    std::shared_ptr<dai::ImgFrame> getLatest() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _latest;
    }

private:
    mutable std::mutex _mutex;
    std::shared_ptr<dai::ImgFrame> _latest;
};

struct HandTrackingSetup {
    bool enabled = false;
    saiHandTracking::Endpoints endpoints;
    saiHandTracking::Settings settings;
    saiHandTracking::Intrinsics intrinsics;
};

struct PipelineWrapper {
    PipelineWrapper(
        std::shared_ptr<spectacularAI::daiPlugin::Pipeline> handle,
        std::shared_ptr<dai::Pipeline> pipeline,
        std::shared_ptr<dai::Device> device,
        std::shared_ptr<ColorFrameQueue> colorFrames,
        std::shared_ptr<dai::DataOutputQueue> colorOutput,
        HandTrackingSetup handTracking) :
        _handle(handle), _pipeline(pipeline), _device(device),
        _colorFrames(colorFrames), _colorOutput(colorOutput),
        _handTracking(std::move(handTracking)) {};
    const std::shared_ptr<spectacularAI::daiPlugin::Pipeline> getHandle() const { return _handle; }
    const std::shared_ptr<dai::Device> getDevice() const { return _device; }
    const std::shared_ptr<ColorFrameQueue> getColorFrames() const { return _colorFrames; }
    const std::shared_ptr<dai::DataOutputQueue> getColorOutput() const { return _colorOutput; }
    const HandTrackingSetup& getHandTracking() const { return _handTracking; }

private:
    const std::shared_ptr<spectacularAI::daiPlugin::Pipeline> _handle;
    const std::shared_ptr<dai::Pipeline> _pipeline;
    const std::shared_ptr<dai::Device> _device;
    const std::shared_ptr<ColorFrameQueue> _colorFrames;
    const std::shared_ptr<dai::DataOutputQueue> _colorOutput;
    const HandTrackingSetup _handTracking;
};

struct SessionWrapper {
    SessionWrapper(
        std::unique_ptr<spectacularAI::daiPlugin::Session> handle,
        std::shared_ptr<ColorFrameQueue> colorFrames,
        std::shared_ptr<dai::DataOutputQueue> colorOutput,
        std::unique_ptr<saiHandTracking::HandTracker> handTracker) :
        _handle(std::move(handle)), _colorFrames(colorFrames), _colorOutput(colorOutput),
        _handTracker(std::move(handTracker)) {};
    spectacularAI::daiPlugin::Session* getHandle() const { return _handle.get(); }
    const std::shared_ptr<ColorFrameQueue> getColorFrames() const { return _colorFrames; }
    const std::shared_ptr<dai::DataOutputQueue> getColorOutput() const { return _colorOutput; }
    saiHandTracking::HandTracker* getHandTracker() const { return _handTracker.get(); }

private:
    std::unique_ptr<spectacularAI::daiPlugin::Session> _handle;
    const std::shared_ptr<ColorFrameQueue> _colorFrames;
    const std::shared_ptr<dai::DataOutputQueue> _colorOutput;
    // Stops (joins) the worker thread before the other members are destroyed
    std::unique_ptr<saiHandTracking::HandTracker> _handTracker;
};

struct ColorFrameWrapper {
    explicit ColorFrameWrapper(std::shared_ptr<dai::ImgFrame> frame) : _frame(frame) {};
    const std::shared_ptr<dai::ImgFrame> getHandle() const { return _frame; }

private:
    const std::shared_ptr<dai::ImgFrame> _frame;
};

extern "C"
{
    /** Pipeline API */
    EXPORT_API PipelineWrapper* sai_depthai_pipeline_build(
        ConfigurationWrapper* configuration,
        const char** internalParameters,
        int internalParametersCount,
        callback_t_mapper_output onMapperOutput);
    /** Message of the last error of a failed call of this thread. Returns the length of the message. */
    EXPORT_API int sai_depthai_get_last_error(char* buffer, int bufferSize);
    EXPORT_API SessionWrapper* sai_depthai_pipeline_start_session(PipelineWrapper* pipelineHandle, char* errorMsg);
    EXPORT_API void sai_depthai_pipeline_release(PipelineWrapper* pipelineHandle);
    /** Prints device CPU, memory and temperature to the native log (diagnostics) */
    EXPORT_API void sai_depthai_pipeline_print_device_stats(PipelineWrapper* pipelineHandle);

    /** Session API */
    EXPORT_API bool sai_depthai_session_has_output(const SessionWrapper* sessionHandle);
    EXPORT_API VioOutputWrapper* sai_depthai_session_get_output(SessionWrapper* sessionHandle);
    EXPORT_API VioOutputWrapper* sai_depthai_session_wait_for_output(SessionWrapper* sessionHandle);
    EXPORT_API ColorFrameWrapper* sai_depthai_session_get_color_frame(const SessionWrapper* sessionHandle);
    EXPORT_API unsigned int sai_color_frame_get_width(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API unsigned int sai_color_frame_get_height(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API int64_t sai_color_frame_get_sequence_number(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API double sai_color_frame_get_timestamp(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API const uint8_t* sai_color_frame_get_data(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API unsigned int sai_color_frame_get_data_size(const ColorFrameWrapper* colorFrameHandle);
    EXPORT_API void sai_color_frame_release(const ColorFrameWrapper* colorFrameHandle);

    /**
     * Hand tracking API. Hand tracking runs on its own thread, these functions never block.
     *
     * sai_depthai_session_get_hand_frame copies the latest hand tracking result when it is newer than
     * afterSequence. Returns the number of hands (>= 0), -1 when there is no newer result and -2 when hand
     * tracking is not enabled. The buffer receives one block of sai_hand_tracking_float_stride() floats per
     * hand. Layout of a block (21 landmarks x 3 floats each):
     *   [0] id  [1] landmark score  [2] handedness (> 0.5 right hand)  [3] gesture  [4] ROI rotation (rad)
     *   [5] hand scale (user's hand size relative to the model)  [6] stereo depth at palm (m, 0 = none)
     *   [7] placed (1 when the camera landmarks are valid)
     *   [8..70]    image landmarks: x, y normalized in the color image, z relative
     *   [71..133]  camera landmarks: meters, camera frame (x right, y down, z forward)
     *   [134..196] hand centered landmarks in meters, image aligned axes
     * deviceTimestamp is in seconds on the same clock as the VIO pose times.
     */
    EXPORT_API int sai_hand_tracking_float_stride();
    EXPORT_API int sai_depthai_session_get_hand_frame(
        const SessionWrapper* sessionHandle,
        int64_t afterSequence,
        int64_t* sequence,
        double* deviceTimestamp,
        double* latencyMs,
        float* buffer,
        int bufferFloats);
    /** Color image size and intrinsics (fx, fy, cx, cy) used for the camera landmarks */
    EXPORT_API bool sai_depthai_session_get_hand_info(
        const SessionWrapper* sessionHandle,
        float* intrinsics,
        int* width,
        int* height);
    /**
     * Health of the hand tracking: chip temperature (Celsius), work reduction level (0 = none, 1..3 = the device is
     * hot and the tracking is slowed down) and whether the networks stopped answering. False without hand tracking.
     */
    EXPORT_API bool sai_depthai_session_get_hand_status(
        const SessionWrapper* sessionHandle,
        float* chipTemperature,
        int* throttleLevel,
        bool* stalled);
    /** True when an offline replay (SAI_HAND_REPLAY) has processed all frames */
    EXPORT_API bool sai_depthai_session_hand_replay_finished(const SessionWrapper* sessionHandle);

    EXPORT_API void sai_depthai_session_add_trigger(
        SessionWrapper* sessionHandle,
        double t,
        int tag);
    EXPORT_API void sai_depthai_session_add_absolute_pose(
        SessionWrapper* sessionHandle,
        spectacularAI::Pose pose,
        Matrix3dWrapper positionCovariance,
        double orientationVariance);
    EXPORT_API spectacularAI::CameraPose* sai_depthai_session_get_rgb_camera_pose(
        SessionWrapper* sessionHandle,
        const VioOutputWrapper* vioOutputHandle);
    EXPORT_API void sai_depthai_session_release(SessionWrapper* sessionHandle);
}
