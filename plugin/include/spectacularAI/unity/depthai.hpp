#pragma once

#include <spectacularAI/depthai/plugin.hpp>
#include "types.hpp"
#include "mapping.hpp"
#include "output.hpp"
#include "hand_tracking.hpp"

#include <cstdint>
#include <mutex>

namespace dai {
class DataOutputQueue;
}

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
    bool enableHandTracking=false;
    const char* handTrackingPalmModelPath="";
};

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

struct PipelineWrapper {
    PipelineWrapper(
        std::shared_ptr<spectacularAI::daiPlugin::Pipeline> handle,
        std::shared_ptr<dai::Pipeline> pipeline,
        std::shared_ptr<dai::Device> device,
        std::shared_ptr<ColorFrameQueue> colorFrames,
        std::shared_ptr<dai::DataOutputQueue> colorOutput,
        std::shared_ptr<dai::DataOutputQueue> handTrackingOutput) :
        _handle(handle), _pipeline(pipeline), _device(device),
        _colorFrames(colorFrames), _colorOutput(colorOutput),
        _handTrackingOutput(handTrackingOutput) {};
    const std::shared_ptr<spectacularAI::daiPlugin::Pipeline> getHandle() const { return _handle; }
    const std::shared_ptr<dai::Device> getDevice() const { return _device; }
    const std::shared_ptr<ColorFrameQueue> getColorFrames() const { return _colorFrames; }
    const std::shared_ptr<dai::DataOutputQueue> getColorOutput() const { return _colorOutput; }
    const std::shared_ptr<dai::DataOutputQueue> getHandTrackingOutput() const { return _handTrackingOutput; }

private:
    const std::shared_ptr<spectacularAI::daiPlugin::Pipeline> _handle;
    const std::shared_ptr<dai::Pipeline> _pipeline;
    const std::shared_ptr<dai::Device> _device;
    const std::shared_ptr<ColorFrameQueue> _colorFrames;
    const std::shared_ptr<dai::DataOutputQueue> _colorOutput;
    const std::shared_ptr<dai::DataOutputQueue> _handTrackingOutput;
};

struct SessionWrapper {
    SessionWrapper(
        std::unique_ptr<spectacularAI::daiPlugin::Session> handle,
        std::shared_ptr<ColorFrameQueue> colorFrames,
        std::shared_ptr<dai::DataOutputQueue> colorOutput,
        std::shared_ptr<dai::DataOutputQueue> handTrackingOutput) :
        _handle(std::move(handle)), _colorFrames(colorFrames),
        _colorOutput(colorOutput), _handTrackingOutput(handTrackingOutput) {};
    spectacularAI::daiPlugin::Session* getHandle() const { return _handle.get(); }
    const std::shared_ptr<ColorFrameQueue> getColorFrames() const { return _colorFrames; }
    const std::shared_ptr<dai::DataOutputQueue> getColorOutput() const { return _colorOutput; }
    const std::shared_ptr<dai::DataOutputQueue> getHandTrackingOutput() const { return _handTrackingOutput; }

private:
    std::unique_ptr<spectacularAI::daiPlugin::Session> _handle;
    const std::shared_ptr<ColorFrameQueue> _colorFrames;
    const std::shared_ptr<dai::DataOutputQueue> _colorOutput;
    const std::shared_ptr<dai::DataOutputQueue> _handTrackingOutput;
};

struct ColorFrameWrapper {
    explicit ColorFrameWrapper(std::shared_ptr<dai::ImgFrame> frame) : _frame(frame) {};
    const std::shared_ptr<dai::ImgFrame> getHandle() const { return _frame; }

private:
    const std::shared_ptr<dai::ImgFrame> _frame;
};

struct HandTrackingOutputWrapper {
    HandTrackingOutputWrapper(
        std::vector<saiHandTracking::PalmDetection> detections,
        int64_t sequenceNumber,
        double timestamp) :
        _detections(std::move(detections)),
        _sequenceNumber(sequenceNumber),
        _timestamp(timestamp) {};
    const std::vector<saiHandTracking::PalmDetection>& getDetections() const { return _detections; }
    int64_t getSequenceNumber() const { return _sequenceNumber; }
    double getTimestamp() const { return _timestamp; }

private:
    const std::vector<saiHandTracking::PalmDetection> _detections;
    const int64_t _sequenceNumber;
    const double _timestamp;
};

extern "C"
{
    /** Pipeline API */
    EXPORT_API PipelineWrapper* sai_depthai_pipeline_build(
        ConfigurationWrapper* configuration,
        const char** internalParameters,
        int internalParametersCount,
        callback_t_mapper_output onMapperOutput,
        char* errorMsg);
    EXPORT_API SessionWrapper* sai_depthai_pipeline_start_session(PipelineWrapper* pipelineHandle, char* errorMsg);
    EXPORT_API void sai_depthai_pipeline_release(PipelineWrapper* pipelineHandle);

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
    EXPORT_API HandTrackingOutputWrapper* sai_depthai_session_get_hand_tracking_output(
        const SessionWrapper* sessionHandle);
    EXPORT_API int sai_hand_tracking_output_get_count(const HandTrackingOutputWrapper* outputHandle);
    EXPORT_API int64_t sai_hand_tracking_output_get_sequence_number(const HandTrackingOutputWrapper* outputHandle);
    EXPORT_API double sai_hand_tracking_output_get_timestamp(const HandTrackingOutputWrapper* outputHandle);
    EXPORT_API float sai_hand_tracking_output_get_score(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex);
    EXPORT_API float sai_hand_tracking_output_get_box_value(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex,
        int valueIndex);
    EXPORT_API void sai_hand_tracking_output_release(const HandTrackingOutputWrapper* outputHandle);
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
