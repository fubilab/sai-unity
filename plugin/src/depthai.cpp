#include "../include/spectacularAI/unity/depthai.hpp"

#include <string>
#include <depthai/depthai.hpp>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <cassert>
#include <cstring>
#include <stdexcept>

namespace {

void create_configuration(
        const ConfigurationWrapper &w,
        const char** internalParameters,
        int internalParametersCount,
        spectacularAI::daiPlugin::Configuration &config) {
    config.useStereo = w.useStereo;
    config.useSlam = w.useSlam;
    config.useFeatureTracker = w.useFeatureTracker;
    config.fastVio = w.fastVio;
    config.useColorStereoCameras = w.useColorStereoCameras;
    config.mapSavePath = w.mapSavePath;
    config.mapLoadPath = w.mapLoadPath;
    config.aprilTagPath = w.aprilTagPath;
    config.accFrequencyHz = w.accFrequencyHz;
    config.gyroFrequencyHz = w.gyroFrequencyHz;
    config.keyframeCandidateEveryNthFrame = w.keyframeCandidateEveryNthFrame;
    config.inputResolution = w.inputResolution;
    config.recordingFolder = w.recordingFolder;
    config.recordingOnly = w.recordingOnly;
    config.fastImu = w.fastImu;
    config.lowLatency = w.lowLatency;
    config.useColor = w.useColor;

    for (int i = 0; i < internalParametersCount; ++i) {
        std::string k = std::string(internalParameters[2 * i]);
        std::string v = std::string(internalParameters[2 * i + 1]);
        config.internalParameters.insert(std::make_pair(k, v));
    }
}

} // anonymous namespace

PipelineWrapper* sai_depthai_pipeline_build(
        ConfigurationWrapper* configuration,
        const char** internalParameters,
        int internalParametersCount,
        callback_t_mapper_output onMapperOutput) {
    std::shared_ptr<dai::Pipeline> pipeline = std::make_shared<dai::Pipeline>();
    std::shared_ptr<ColorFrameQueue> colorFrames = std::make_shared<ColorFrameQueue>();

    spectacularAI::daiPlugin::Configuration config;
    create_configuration(*configuration, internalParameters, internalParametersCount, config);

    std::shared_ptr<spectacularAI::daiPlugin::Pipeline> handle = onMapperOutput ?
        std::make_shared<spectacularAI::daiPlugin::Pipeline>(*pipeline, config,
            [onMapperOutput](spectacularAI::mapping::MapperOutputPtr mapperOutput) {
                onMapperOutput(new MapperOutputWrapper(mapperOutput));
            })
        : std::make_shared<spectacularAI::daiPlugin::Pipeline>(*pipeline, config);
    if (configuration->enableHandTracking) {
        handle->color->setInterleaved(false);
        handle->hooks.color = [colorFrames](std::shared_ptr<dai::ImgFrame> frame) {
            colorFrames->push(frame);
        };
    }

    bool handTrackingPipelineEnabled = configuration->enableHandTracking &&
        configuration->handTrackingPalmModelPath != nullptr &&
        configuration->handTrackingPalmModelPath[0] != '\0';
    if (handTrackingPipelineEnabled) {
        auto palmInput = pipeline->create<dai::node::ImageManip>();
        palmInput->initialConfig.setResize(128, 128);
        palmInput->setMaxOutputFrameSize(128 * 128 * 3);
        handle->color->preview.link(palmInput->inputImage);

        auto palmNetwork = pipeline->create<dai::node::NeuralNetwork>();
        palmNetwork->setBlobPath(configuration->handTrackingPalmModelPath);
        palmInput->out.link(palmNetwork->input);

        auto palmOutput = pipeline->create<dai::node::XLinkOut>();
        palmOutput->setStreamName("sai_hand_palm");
        palmOutput->input.setBlocking(false);
        palmOutput->input.setQueueSize(1);
        palmNetwork->out.link(palmOutput->input);
    }

    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
    std::shared_ptr<dai::DataOutputQueue> handTrackingOutput = handTrackingPipelineEnabled ?
        device->getOutputQueue("sai_hand_palm", 1, false) : nullptr;
    return new PipelineWrapper(handle, pipeline, device, colorFrames, handTrackingOutput);
}

SessionWrapper* sai_depthai_pipeline_start_session(PipelineWrapper* pipelineHandle, char* errorMsg) {
    assert(pipelineHandle);
    try {
        return new SessionWrapper(
            pipelineHandle->getHandle()->startSession(*pipelineHandle->getDevice()),
            pipelineHandle->getColorFrames(),
            pipelineHandle->getHandTrackingOutput());
    } catch(const std::runtime_error &e) {
        if (errorMsg != nullptr) {
            strncpy(errorMsg, e.what(), 1000 - 1);
            errorMsg[1000 - 1] = '\0'; // Ensure null-termination
        } else {
            throw e;
        }
    }

    return nullptr;
}

void sai_depthai_pipeline_release(PipelineWrapper* pipelineHandle) {
    if (pipelineHandle) delete pipelineHandle;
}

bool sai_depthai_session_has_output(const SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    return sessionHandle->getHandle()->hasOutput();
}

VioOutputWrapper* sai_depthai_session_get_output(SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    return new VioOutputWrapper(sessionHandle->getHandle()->getOutput());
}

VioOutputWrapper* sai_depthai_session_wait_for_output(SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    return new VioOutputWrapper(sessionHandle->getHandle()->waitForOutput());
}

ColorFrameWrapper* sai_depthai_session_get_color_frame(const SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    std::shared_ptr<dai::ImgFrame> frame = sessionHandle->getColorFrames()->getLatest();
    return frame ? new ColorFrameWrapper(frame) : nullptr;
}

unsigned int sai_color_frame_get_width(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return colorFrameHandle->getHandle()->getWidth();
}

unsigned int sai_color_frame_get_height(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return colorFrameHandle->getHandle()->getHeight();
}

int64_t sai_color_frame_get_sequence_number(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return colorFrameHandle->getHandle()->getSequenceNum();
}

double sai_color_frame_get_timestamp(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return std::chrono::duration<double>(
        colorFrameHandle->getHandle()->getTimestampDevice().time_since_epoch()).count();
}

const uint8_t* sai_color_frame_get_data(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return colorFrameHandle->getHandle()->getData().data();
}

unsigned int sai_color_frame_get_data_size(const ColorFrameWrapper* colorFrameHandle) {
    assert(colorFrameHandle);
    return static_cast<unsigned int>(colorFrameHandle->getHandle()->getData().size());
}

void sai_color_frame_release(const ColorFrameWrapper* colorFrameHandle) {
    if (colorFrameHandle) delete colorFrameHandle;
}

HandTrackingOutputWrapper* sai_depthai_session_get_hand_tracking_output(
        const SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    const auto outputQueue = sessionHandle->getHandTrackingOutput();
    if (!outputQueue) return nullptr;

    std::shared_ptr<dai::NNData> inference;
    while (auto nextInference = outputQueue->tryGet<dai::NNData>()) {
        inference = nextInference;
    }
    if (!inference) return nullptr;

    std::vector<float> scores = inference->getLayerFp16("classificators");
    std::vector<float> regressors = inference->getLayerFp16("regressors");
    if (scores.empty() || regressors.empty()) return nullptr;

    return new HandTrackingOutputWrapper(
        saiHandTracking::decodePalmDetections(scores, regressors, 0.5f, false),
        inference->getSequenceNum(),
        std::chrono::duration<double>(
            inference->getTimestampDevice().time_since_epoch()).count());
}

int sai_hand_tracking_output_get_count(const HandTrackingOutputWrapper* outputHandle) {
    assert(outputHandle);
    return static_cast<int>(outputHandle->getDetections().size());
}

int64_t sai_hand_tracking_output_get_sequence_number(const HandTrackingOutputWrapper* outputHandle) {
    assert(outputHandle);
    return outputHandle->getSequenceNumber();
}

double sai_hand_tracking_output_get_timestamp(const HandTrackingOutputWrapper* outputHandle) {
    assert(outputHandle);
    return outputHandle->getTimestamp();
}

float sai_hand_tracking_output_get_score(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).score;
}

float sai_hand_tracking_output_get_box_value(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex,
        int valueIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).box.at(valueIndex);
}

void sai_hand_tracking_output_release(const HandTrackingOutputWrapper* outputHandle) {
    if (outputHandle) delete outputHandle;
}

void sai_depthai_session_add_trigger(
        SessionWrapper* sessionHandle,
        double t,
        int tag) {
    assert(sessionHandle);
    sessionHandle->getHandle()->addTrigger(t, tag);
}

void sai_depthai_session_add_absolute_pose(
        SessionWrapper* sessionHandle,
        spectacularAI::Pose pose,
        Matrix3dWrapper positionCovariance,
        double orientationVariance) {
    assert(sessionHandle);
    sessionHandle->getHandle()->addAbsolutePose(
        pose,
        reinterpret_cast<spectacularAI::Matrix3d&>(positionCovariance),
        orientationVariance);
}

spectacularAI::CameraPose* sai_depthai_session_get_rgb_camera_pose(
    SessionWrapper* sessionHandle,
        const VioOutputWrapper* vioOutputHandle) {
    assert(sessionHandle);
    spectacularAI::CameraPose* cameraPose = new spectacularAI::CameraPose();
    *cameraPose = sessionHandle->getHandle()->getRgbCameraPose(*vioOutputHandle->getHandle());
    return cameraPose;
}

void sai_depthai_session_release(SessionWrapper* sessionHandle) {
    if (sessionHandle) delete sessionHandle;
}
