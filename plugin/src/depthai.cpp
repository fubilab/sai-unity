#include "../include/spectacularAI/unity/depthai.hpp"

#include <string>
#include <depthai/depthai.hpp>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

bool envEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && std::strcmp(value, "0") != 0;
}

int envInt(const char* name, int defaultValue) {
    const char* value = std::getenv(name);
    return (value != nullptr && value[0] != '\0') ? std::atoi(value) : defaultValue;
}

std::string envString(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr ? std::string(value) : std::string();
}

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
    // The hand tracking needs the color camera
    config.useColor = w.useColor || w.enableHandTracking;

    for (int i = 0; i < internalParametersCount; ++i) {
        std::string k = std::string(internalParameters[2 * i]);
        std::string v = std::string(internalParameters[2 * i + 1]);
        config.internalParameters.insert(std::make_pair(k, v));
    }
}

// Error of the last failed call of this thread, see sai_depthai_get_last_error
thread_local std::string g_lastError;

void copyError(char* errorMsg, const char* message) {
    g_lastError = message;
    if (errorMsg == nullptr) return;
    strncpy(errorMsg, message, 1000 - 1);
    errorMsg[1000 - 1] = '\0';
}

// Intrinsics of the color preview image, from the device calibration
saiHandTracking::Intrinsics readColorIntrinsics(dai::Device& device, int width, int height) {
    saiHandTracking::Intrinsics intrinsics;
    try {
        const auto matrix = device.readCalibration().getCameraIntrinsics(dai::CameraBoardSocket::CAM_A, width, height);
        intrinsics.fx = matrix[0][0];
        intrinsics.fy = matrix[1][1];
        intrinsics.cx = matrix[0][2];
        intrinsics.cy = matrix[1][2];
    } catch (const std::exception& e) {
        std::cerr << "[sai-hand] cannot read color camera calibration: " << e.what() << std::endl;
    }
    return intrinsics;
}

} // anonymous namespace

PipelineWrapper* sai_depthai_pipeline_build(
        ConfigurationWrapper* configuration,
        const char** internalParameters,
        int internalParametersCount,
        callback_t_mapper_output onMapperOutput) {
    // Same signature as the original plugin, errors are reported with sai_depthai_get_last_error
    char* errorMsg = nullptr;
    g_lastError.clear();
    try {
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

        const bool handTrackingRequested = configuration->enableHandTracking;
        const bool wantColor = configuration->useColor || handTrackingRequested;
        int previewWidth = 0;
        int previewHeight = 0;
        if (wantColor) {
            if (!handle->color) throw std::runtime_error("The color camera is not available in this configuration.");
            handle->color->setInterleaved(false);
            // The color camera is a main heat source of the device. Hand tracking does not need 30 fps.
            const int colorFps = envInt("SAI_HAND_COLOR_FPS", configuration->handTrackingColorFps);
            if (handTrackingRequested && colorFps > 0) handle->color->setFps(static_cast<float>(colorFps));
            previewWidth = handle->color->getPreviewWidth();
            previewHeight = handle->color->getPreviewHeight();
            if (envEnabled("SAI_HAND_TRACKING_DEBUG")) {
                std::clog << "[sai-hand] color camera: fps=" << handle->color->getFps()
                    << " preview=" << previewWidth << "x" << previewHeight
                    << " isp=" << handle->color->getIspWidth() << "x" << handle->color->getIspHeight() << std::endl;
            }

            // In offline replay nobody reads the live preview. Do not stream it (diagnostics, SAI_HAND_NO_PREVIEW).
            if (!envEnabled("SAI_HAND_NO_PREVIEW")) {
                auto colorOutputNode = pipeline->create<dai::node::XLinkOut>();
                colorOutputNode->setStreamName("sai_color");
                colorOutputNode->input.setBlocking(false);
                colorOutputNode->input.setQueueSize(1);
                handle->color->preview.link(colorOutputNode->input);
            }
        }

        const bool handTrackingEnabled = handTrackingRequested;
        if (handTrackingEnabled &&
            (configuration->handTrackingPalmModelPath == nullptr || configuration->handTrackingPalmModelPath[0] == '\0' ||
             configuration->handTrackingLandmarkModelPath == nullptr || configuration->handTrackingLandmarkModelPath[0] == '\0')) {
            throw std::runtime_error("Hand tracking requires the palm detection and hand landmark model paths.");
        }

        bool depthEnabled = false;
        if (handTrackingEnabled) {
            // Palm detection: host sends the (letterboxed) color frame, receives the raw detections
            auto palmInput = pipeline->create<dai::node::XLinkIn>();
            palmInput->setStreamName("sai_hand_palm_input");
            auto palmNetwork = pipeline->create<dai::node::NeuralNetwork>();
            palmNetwork->setBlobPath(configuration->handTrackingPalmModelPath);
            palmNetwork->setNumInferenceThreads(envInt("SAI_HAND_PALM_THREADS", 1));
            if (envInt("SAI_HAND_PALM_NCE", 0) > 0) palmNetwork->setNumNCEPerInferenceThread(envInt("SAI_HAND_PALM_NCE", 0));
            palmInput->out.link(palmNetwork->input);
            auto palmOutput = pipeline->create<dai::node::XLinkOut>();
            palmOutput->setStreamName("sai_hand_palm");
            palmNetwork->out.link(palmOutput->input);

            // Hand landmarks: host sends the cropped and rotated hand image(s)
            auto landmarkInput = pipeline->create<dai::node::XLinkIn>();
            landmarkInput->setStreamName("sai_hand_landmark_input");
            auto landmarkNetwork = pipeline->create<dai::node::NeuralNetwork>();
            landmarkNetwork->setBlobPath(configuration->handTrackingLandmarkModelPath);
            landmarkNetwork->setNumInferenceThreads(envInt("SAI_HAND_LM_THREADS", 2));
            if (envInt("SAI_HAND_LM_NCE", 0) > 0) landmarkNetwork->setNumNCEPerInferenceThread(envInt("SAI_HAND_LM_NCE", 0));
            landmarkInput->out.link(landmarkNetwork->input);
            auto landmarkOutput = pipeline->create<dai::node::XLinkOut>();
            landmarkOutput->setStreamName("sai_hand_landmark");
            landmarkNetwork->out.link(landmarkOutput->input);

            // Depth aligned to the color camera, used to calibrate the user's hand size
            depthEnabled = configuration->handTrackingUseDepth && configuration->useStereo &&
                !configuration->useColorStereoCameras;
            if (depthEnabled && (!handle->monoLeft || !handle->monoRight)) {
                std::cerr << "[sai-hand] stereo cameras not available, hand depth disabled" << std::endl;
                depthEnabled = false;
            }
            if (depthEnabled) {
                auto stereo = pipeline->create<dai::node::StereoDepth>();
                stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
                stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
                stereo->setLeftRightCheck(true);
                stereo->setOutputSize(previewWidth, previewHeight);
                handle->monoLeft->out.link(stereo->left);
                handle->monoRight->out.link(stereo->right);
                auto depthOutput = pipeline->create<dai::node::XLinkOut>();
                depthOutput->setStreamName("sai_hand_depth");
                depthOutput->input.setBlocking(false);
                depthOutput->input.setQueueSize(2);
                stereo->depth.link(depthOutput->input);
            }
        }

        if (envEnabled("SAI_HAND_BENCH")) {
            auto benchIn = pipeline->create<dai::node::XLinkIn>();
            benchIn->setStreamName("sai_bench_in");
            auto benchOut = pipeline->create<dai::node::XLinkOut>();
            benchOut->setStreamName("sai_bench_out");
            benchIn->out.link(benchOut->input);
        }

        if (envEnabled("SAI_HAND_DUMP_NODES")) {
            const auto schema = pipeline->getPipelineSchema(dai::SerializationType::JSON);
            for (const auto& entry : schema.nodes) {
                const auto& node = entry.second;
                std::string properties(node.properties.begin(), node.properties.end());
                if (properties.size() > 1500) properties.resize(1500);
                std::clog << "[sai-hand] node " << node.id << " " << node.name << " " << properties << std::endl;
            }
        }

        if (envEnabled("SAI_HAND_TRACKING_DEBUG")) {
            std::clog << "[sai-hand] pipeline nodes:";
            for (const auto& node : pipeline->getAllNodes()) std::clog << " " << node->getName();
            std::clog << std::endl;
        }

        std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
        std::shared_ptr<dai::DataOutputQueue> colorOutput =
            (wantColor && !envEnabled("SAI_HAND_NO_PREVIEW")) ? device->getOutputQueue("sai_color", 1, false) : nullptr;

        HandTrackingSetup handTracking;
        if (handTrackingEnabled) {
            if (depthEnabled && configuration->handTrackingIrProjectorIntensity > 0.0f) {
                if (!device->setIrLaserDotProjectorIntensity(configuration->handTrackingIrProjectorIntensity)) {
                    std::cerr << "[sai-hand] cannot enable the IR dot projector" << std::endl;
                }
            }
            handTracking.enabled = true;
            handTracking.endpoints.color = colorOutput;
            handTracking.endpoints.palmInput = device->getInputQueue("sai_hand_palm_input", 4, false);
            handTracking.endpoints.palmOutput = device->getOutputQueue("sai_hand_palm", 4, false);
            handTracking.endpoints.landmarkInput = device->getInputQueue("sai_hand_landmark_input", 4, false);
            handTracking.endpoints.landmarkOutput = device->getOutputQueue("sai_hand_landmark", 4, false);
            if (depthEnabled) handTracking.endpoints.depth = device->getOutputQueue("sai_hand_depth", 2, false);
            handTracking.intrinsics = readColorIntrinsics(*device, previewWidth, previewHeight);
            {
                std::shared_ptr<dai::Device> deviceForTemperature = device;
                handTracking.endpoints.chipTemperature = [deviceForTemperature]() {
                    return deviceForTemperature->getChipTemperature().average;
                };
            }
            handTracking.settings.searchIntervalMs =
                envInt("SAI_HAND_SEARCH_INTERVAL_MS", handTracking.settings.searchIntervalMs);
            handTracking.settings.searchCrops = envInt("SAI_HAND_SEARCH_CROPS", 1) != 0;
            handTracking.settings.rejectionMemoryMs = envInt("SAI_HAND_REJECT_MEMORY_MS", handTracking.settings.rejectionMemoryMs);
            if (const char* threshold = std::getenv("SAI_HAND_PALM_THRESHOLD")) {
                // percent, for experiments: a lower threshold gives more candidates
                handTracking.settings.palmScoreThreshold = std::max(0.05f, std::min(0.95f, static_cast<float>(std::atoi(threshold)) / 100.0f));
            }
            handTracking.settings.throttleStartC = static_cast<float>(
                envInt("SAI_HAND_THROTTLE_START_C", static_cast<int>(handTracking.settings.throttleStartC)));
            handTracking.settings.maxHands = std::max(1, std::min(2, configuration->handTrackingMaxHands));
            handTracking.settings.solo = handTracking.settings.maxHands == 1;
            // Testing and diagnostics, see README
            handTracking.settings.replayPath = envString("SAI_HAND_REPLAY");
            handTracking.settings.replayRealtime = envEnabled("SAI_HAND_REPLAY_REALTIME");
            handTracking.settings.replayLoop = envEnabled("SAI_HAND_REPLAY_LOOP");
            handTracking.settings.logPath = envString("SAI_HAND_LOG");
            if (envEnabled("SAI_HAND_TRACKING_DEBUG")) {
                std::clog << "[sai-hand] color intrinsics fx=" << handTracking.intrinsics.fx
                    << " fy=" << handTracking.intrinsics.fy << " cx=" << handTracking.intrinsics.cx
                    << " cy=" << handTracking.intrinsics.cy << " depth=" << (depthEnabled ? "on" : "off") << std::endl;
            }
        }

        return new PipelineWrapper(handle, pipeline, device, colorFrames, colorOutput, std::move(handTracking));
    } catch (const std::exception &e) {
        copyError(errorMsg, e.what());
    } catch (...) {
        copyError(errorMsg, "Unknown native exception while building the DepthAI pipeline.");
    }

    return nullptr;
}

SessionWrapper* sai_depthai_pipeline_start_session(PipelineWrapper* pipelineHandle, char* errorMsg) {
    if (pipelineHandle == nullptr) {
        copyError(errorMsg, "The pipeline was not created.");
        return nullptr;
    }
    g_lastError.clear();
    try {
        std::unique_ptr<spectacularAI::daiPlugin::Session> session =
            pipelineHandle->getHandle()->startSession(*pipelineHandle->getDevice());

        if (envEnabled("SAI_HAND_BENCH")) {
            // Host -> device -> host round trip time for different message sizes under the real pipeline load
            dai::Device& device = *pipelineHandle->getDevice();
            auto input = device.getInputQueue("sai_bench_in", 4, false);
            auto output = device.getOutputQueue("sai_bench_out", 4, false);
            for (std::size_t size : {1024u, 49152u, 150528u, 300000u}) {
                std::vector<double> times;
                for (int i = 0; i < 60; ++i) {
                    auto message = std::make_shared<dai::Buffer>();
                    message->setData(std::vector<std::uint8_t>(size, 1));
                    const auto start = std::chrono::steady_clock::now();
                    input->send(message);
                    bool timedOut = false;
                    output->get<dai::Buffer>(std::chrono::milliseconds(2000), timedOut);
                    if (timedOut) break;
                    times.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
                }
                if (times.size() > 10) {
                    std::sort(times.begin(), times.end());
                    std::clog << "[sai-hand] bench loopback " << size << " bytes: median " << times[times.size() / 2]
                        << " ms, p90 " << times[times.size() * 9 / 10] << " ms, max " << times.back() << " ms" << std::endl;
                }
            }
        }

        std::unique_ptr<saiHandTracking::HandTracker> handTracker;
        const HandTrackingSetup& handTracking = pipelineHandle->getHandTracking();
        std::shared_ptr<ColorFrameQueue> colorFrames = pipelineHandle->getColorFrames();
        if (handTracking.enabled) {
            handTracker.reset(new saiHandTracking::HandTracker(
                handTracking.settings,
                handTracking.endpoints,
                handTracking.intrinsics,
                [colorFrames](std::shared_ptr<dai::ImgFrame> frame) { colorFrames->push(frame); }));
        }
        return new SessionWrapper(
            std::move(session), colorFrames, pipelineHandle->getColorOutput(), std::move(handTracker));
    } catch(const std::exception &e) {
        copyError(errorMsg, e.what());
    } catch (...) {
        copyError(errorMsg, "Unknown native exception while starting the DepthAI session.");
    }

    return nullptr;
}

void sai_depthai_pipeline_print_device_stats(PipelineWrapper* pipelineHandle) {
    assert(pipelineHandle);
    try {
        dai::Device& device = *pipelineHandle->getDevice();
        const auto css = device.getLeonCssCpuUsage();
        const auto mss = device.getLeonMssCpuUsage();
        const auto ddr = device.getDdrMemoryUsage();
        const auto cmx = device.getCmxMemoryUsage();
        const auto temperature = device.getChipTemperature();
        std::clog << "[sai-hand] device: leonCSS " << css.average * 100.0f << "% leonMSS " << mss.average * 100.0f
            << "% ddr " << ddr.used / (1024 * 1024) << "/" << ddr.total / (1024 * 1024) << " MB"
            << " cmx " << cmx.used / 1024 << "/" << cmx.total / 1024 << " kB"
            << " temp " << temperature.average << " C" << std::endl;
    } catch (const std::exception& e) {
        std::clog << "[sai-hand] device stats failed: " << e.what() << std::endl;
    }
}

int sai_depthai_get_last_error(char* buffer, int bufferSize) {
    if (buffer == nullptr || bufferSize <= 0) return static_cast<int>(g_lastError.size());
    strncpy(buffer, g_lastError.c_str(), static_cast<std::size_t>(bufferSize) - 1);
    buffer[bufferSize - 1] = '\0';
    return static_cast<int>(g_lastError.size());
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
    // With hand tracking the worker thread reads the color queue. Otherwise read it here.
    if (!sessionHandle->getHandTracker()) {
        if (const auto colorOutput = sessionHandle->getColorOutput()) {
            std::shared_ptr<dai::ImgFrame> frame;
            while (auto nextFrame = colorOutput->tryGet<dai::ImgFrame>()) {
                frame = nextFrame;
            }
            if (frame) sessionHandle->getColorFrames()->push(frame);
        }
    }

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

int sai_hand_tracking_float_stride() {
    return saiHandTracking::kHandFloatStride;
}

int sai_depthai_session_get_hand_frame(
        const SessionWrapper* sessionHandle,
        int64_t afterSequence,
        int64_t* sequence,
        double* deviceTimestamp,
        double* latencyMs,
        float* buffer,
        int bufferFloats) {
    assert(sessionHandle);
    const saiHandTracking::HandTracker* tracker = sessionHandle->getHandTracker();
    if (tracker == nullptr) return -2;
    const std::shared_ptr<const saiHandTracking::HandFrame> frame = tracker->getLatest();
    if (!frame || frame->sequence <= afterSequence) return -1;

    if (sequence) *sequence = frame->sequence;
    if (deviceTimestamp) *deviceTimestamp = frame->deviceTimestamp;
    if (latencyMs) *latencyMs = frame->latencyMs;

    const int stride = saiHandTracking::kHandFloatStride;
    const int capacity = buffer != nullptr ? bufferFloats / stride : 0;
    const int count = std::min(static_cast<int>(frame->hands.size()), capacity);
    for (int i = 0; i < count; ++i) {
        const saiHandTracking::TrackedHand& hand = frame->hands[i];
        float* out = buffer + i * stride;
        out[0] = static_cast<float>(hand.id);
        out[1] = hand.landmarkScore;
        out[2] = hand.handedness;
        out[3] = static_cast<float>(hand.gesture);
        out[4] = hand.rotation;
        out[5] = hand.handScale;
        out[6] = hand.stereoDepth;
        out[7] = hand.placed ? 1.0f : 0.0f;
        std::copy(hand.imageLandmarks.begin(), hand.imageLandmarks.end(), out + 8);
        std::copy(hand.cameraLandmarks.begin(), hand.cameraLandmarks.end(), out + 8 + 63);
        std::copy(hand.worldLandmarks.begin(), hand.worldLandmarks.end(), out + 8 + 126);
    }
    return count;
}

bool sai_depthai_session_get_hand_info(
        const SessionWrapper* sessionHandle,
        float* intrinsics,
        int* width,
        int* height) {
    assert(sessionHandle);
    const saiHandTracking::HandTracker* tracker = sessionHandle->getHandTracker();
    if (tracker == nullptr) return false;
    const saiHandTracking::Intrinsics& k = tracker->getIntrinsics();
    if (intrinsics) {
        intrinsics[0] = k.fx;
        intrinsics[1] = k.fy;
        intrinsics[2] = k.cx;
        intrinsics[3] = k.cy;
    }
    const std::shared_ptr<dai::ImgFrame> color = sessionHandle->getColorFrames()->getLatest();
    if (width) *width = color ? static_cast<int>(color->getWidth()) : 0;
    if (height) *height = color ? static_cast<int>(color->getHeight()) : 0;
    return k.valid();
}

bool sai_depthai_session_get_hand_status(
        const SessionWrapper* sessionHandle,
        float* chipTemperature,
        int* throttleLevel,
        bool* stalled) {
    assert(sessionHandle);
    const saiHandTracking::HandTracker* tracker = sessionHandle->getHandTracker();
    if (tracker == nullptr) return false;
    const saiHandTracking::HandTrackerStatus status = tracker->getStatus();
    if (chipTemperature) *chipTemperature = status.chipTemperature;
    if (throttleLevel) *throttleLevel = status.throttleLevel;
    if (stalled) *stalled = status.stalled;
    return true;
}

bool sai_depthai_session_hand_replay_finished(const SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    const saiHandTracking::HandTracker* tracker = sessionHandle->getHandTracker();
    return tracker != nullptr && tracker->isReplayFinished();
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
