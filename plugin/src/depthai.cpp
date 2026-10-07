#include "../include/spectacularAI/unity/depthai.hpp"

#include <string>
#include <depthai/depthai.hpp>
#include <depthai/device/DataQueue.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {

struct Point2f {
    float x;
    float y;
};

Point2f landmarkSourcePoint(
        const std::array<Point2f, 4>& points,
        int outputX,
        int outputY,
        int outputSize) {
    const Point2f& origin = points[1];
    const float horizontal = static_cast<float>(outputX) / outputSize;
    const float vertical = static_cast<float>(outputY) / outputSize;
    return {
        origin.x + horizontal * (points[2].x - origin.x) + vertical * (points[3].x - points[2].x),
        origin.y + horizontal * (points[2].y - origin.y) + vertical * (points[3].y - points[2].y)};
}

std::uint8_t samplePlanarChannel(
        const std::vector<std::uint8_t>& data,
        std::size_t planeOffset,
        int width,
        int height,
        float x,
        float y) {
    if (x < 0.0f || y < 0.0f || x > width - 1.0f || y > height - 1.0f) return 0;
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float xWeight = x - x0;
    const float yWeight = y - y0;
    const auto pixel = [&](int pixelX, int pixelY) {
        return static_cast<float>(data[planeOffset + static_cast<std::size_t>(pixelY * width + pixelX)]);
    };
    const float top = pixel(x0, y0) * (1.0f - xWeight) + pixel(x1, y0) * xWeight;
    const float bottom = pixel(x0, y1) * (1.0f - xWeight) + pixel(x1, y1) * xWeight;
    return static_cast<std::uint8_t>(std::lround(top * (1.0f - yWeight) + bottom * yWeight));
}

std::shared_ptr<dai::NNData> createLandmarkInput(
        const std::shared_ptr<dai::ImgFrame>& colorFrame,
    const saiHandTracking::PalmDetection& detection,
    int64_t palmSequence) {
    constexpr int outputSize = 224;
    const int imageWidth = static_cast<int>(colorFrame->getWidth());
    const int imageHeight = static_cast<int>(colorFrame->getHeight());
    const int squareSize = std::max(imageWidth, imageHeight);
    const float padY = static_cast<float>(squareSize - imageHeight) * 0.5f;
    const saiHandTracking::LandmarkRoi roi =
        saiHandTracking::calculateLandmarkRoi(
            detection,
            static_cast<float>(squareSize),
            static_cast<float>(squareSize));
    const float centerX = roi.centerX * squareSize;
    const float centerY = roi.centerY * squareSize;
    const float side = roi.width * squareSize;
    const float angle = roi.angleDegrees * 3.14159265358979323846f / 180.0f;
    const float b = std::cos(angle) * 0.5f;
    const float a = std::sin(angle) * 0.5f;
    const float p0x = centerX - a * side - b * side;
    const float p0y = centerY + b * side - a * side;
    const float p1x = centerX + a * side - b * side;
    const float p1y = centerY - b * side - a * side;
    const std::array<Point2f, 4> points = {{
        {static_cast<float>(static_cast<int>(p0x)), static_cast<float>(static_cast<int>(p0y))},
        {static_cast<float>(static_cast<int>(p1x)), static_cast<float>(static_cast<int>(p1y))},
        {static_cast<float>(static_cast<int>(2.0f * centerX - p0x)), static_cast<float>(static_cast<int>(2.0f * centerY - p0y))},
        {static_cast<float>(static_cast<int>(2.0f * centerX - p1x)), static_cast<float>(static_cast<int>(2.0f * centerY - p1y))}}};

    const std::vector<std::uint8_t>& source = colorFrame->getData();
    const std::size_t pixelCount = static_cast<std::size_t>(imageWidth) * imageHeight;
    std::vector<std::uint8_t> planar(static_cast<std::size_t>(outputSize) * outputSize * 3);
    const char* frameDumpPath = std::getenv("SAI_HAND_TRACKING_FRAME_DUMP");
    static bool frameDumped = false;
    if (frameDumpPath && frameDumpPath[0] != '\0' && !frameDumped) {
        std::ofstream frameDump(frameDumpPath, std::ios::binary);
        frameDump << "P6\n" << imageWidth << ' ' << imageHeight << "\n255\n";
        const bool isBgr = colorFrame->getType() == dai::ImgFrame::Type::BGR888p;
        for (std::size_t index = 0; index < pixelCount; ++index) {
            const char rgb[] = {
                static_cast<char>(source[(isBgr ? 2 : 0) * pixelCount + index]),
                static_cast<char>(source[pixelCount + index]),
                static_cast<char>(source[(isBgr ? 0 : 2) * pixelCount + index])};
            frameDump.write(rgb, sizeof(rgb));
        }
        frameDumped = static_cast<bool>(frameDump);
        if (frameDumped) {
            std::clog << "[sai-hand] source_frame_dump seq="
                << colorFrame->getSequenceNum() << " path=" << frameDumpPath << std::endl;
        }
    }
    constexpr int sourcePlaneForBgr[] = {2, 1, 0};
    for (int outputY = 0; outputY < outputSize; ++outputY) {
        for (int outputX = 0; outputX < outputSize; ++outputX) {
            const Point2f sourcePoint = landmarkSourcePoint(points, outputX, outputY, outputSize);
            const float sourceY = sourcePoint.y - padY;
            const std::size_t outputIndex = static_cast<std::size_t>(outputY * outputSize + outputX);
            for (int channel = 0; channel < 3; ++channel) {
                planar[static_cast<std::size_t>(channel) * outputSize * outputSize + outputIndex] =
                    samplePlanarChannel(
                        source,
                        static_cast<std::size_t>(sourcePlaneForBgr[channel]) * pixelCount,
                        imageWidth,
                        imageHeight,
                        sourcePoint.x,
                        sourceY);
            }
        }
    }
    const char* cropDumpPath = std::getenv("SAI_HAND_TRACKING_CROP_DUMP");
    static bool cropDumped = false;
    if (cropDumpPath && cropDumpPath[0] != '\0' && !cropDumped) {
        std::ofstream cropDump(cropDumpPath, std::ios::binary);
        cropDump << "P6\n" << outputSize << ' ' << outputSize << "\n255\n";
        for (std::size_t index = 0; index < outputSize * outputSize; ++index) {
            const char rgb[] = {
                static_cast<char>(planar[2 * outputSize * outputSize + index]),
                static_cast<char>(planar[outputSize * outputSize + index]),
                static_cast<char>(planar[index])};
            cropDump.write(rgb, sizeof(rgb));
        }
        cropDumped = static_cast<bool>(cropDump);
    }
    static const bool debugEnabled = std::getenv("SAI_HAND_TRACKING_DEBUG") != nullptr;
    static std::size_t debugCount = 0;
    if (debugEnabled && debugCount++ % 10 == 0) {
        const auto range = std::minmax_element(planar.begin(), planar.end());
        const double mean = std::accumulate(planar.begin(), planar.end(), 0.0) / planar.size();
        std::clog << "[sai-hand] landmark_input palm_seq=" << palmSequence
            << " frame_seq=" << colorFrame->getSequenceNum()
            << " palm_score=" << detection.score
            << " box=" << detection.box[0] << ',' << detection.box[1] << ','
            << detection.box[2] << ',' << detection.box[3]
            << " roi_center=" << roi.centerX << ',' << roi.centerY
            << " roi_size=" << roi.width << ',' << roi.height
            << " angle=" << roi.angleDegrees
            << " pixel_min=" << static_cast<int>(*range.first)
            << " pixel_max=" << static_cast<int>(*range.second)
            << " pixel_mean=" << mean << std::endl;
    }
    auto input = std::make_shared<dai::NNData>();
    input->setLayer("input_1", planar);
    return input;
}

std::shared_ptr<dai::ImgFrame> createPalmInput(
        const std::shared_ptr<dai::ImgFrame>& colorFrame) {
    constexpr int outputSize = 128;
    const int imageWidth = static_cast<int>(colorFrame->getWidth());
    const int imageHeight = static_cast<int>(colorFrame->getHeight());
    const float scale = std::min(
        static_cast<float>(outputSize) / imageWidth,
        static_cast<float>(outputSize) / imageHeight);
    const float offsetX = (outputSize - imageWidth * scale) * 0.5f;
    const float offsetY = (outputSize - imageHeight * scale) * 0.5f;
    const std::vector<std::uint8_t>& source = colorFrame->getData();
    const std::size_t pixelCount = static_cast<std::size_t>(imageWidth) * imageHeight;
    std::vector<std::uint8_t> planar(static_cast<std::size_t>(outputSize) * outputSize * 3, 0);
    static const bool logFrameType = std::getenv("SAI_HAND_TRACKING_DEBUG") != nullptr;
    static bool frameTypeLogged = false;
    if (logFrameType && !frameTypeLogged) {
        std::clog << "[sai-hand] color_frame_type="
            << static_cast<int>(colorFrame->getType()) << std::endl;
        frameTypeLogged = true;
    }
    constexpr int sourcePlaneForBgr[] = {2, 1, 0};
    for (int outputY = 0; outputY < outputSize; ++outputY) {
        for (int outputX = 0; outputX < outputSize; ++outputX) {
            const float sourceX = (outputX - offsetX) / scale;
            const float sourceY = (outputY - offsetY) / scale;
            const std::size_t outputIndex = static_cast<std::size_t>(outputY * outputSize + outputX);
            for (int channel = 0; channel < 3; ++channel) {
                planar[static_cast<std::size_t>(channel) * outputSize * outputSize + outputIndex] =
                    samplePlanarChannel(source,
                        static_cast<std::size_t>(sourcePlaneForBgr[channel]) * pixelCount,
                        imageWidth, imageHeight, sourceX, sourceY);
            }
        }
    }
    auto input = std::make_shared<dai::ImgFrame>();
    input->setSequenceNum(colorFrame->getSequenceNum());
    input->setTimestampDevice(colorFrame->getTimestampDevice());
    input->setSize(outputSize, outputSize);
    input->setType(dai::ImgFrame::Type::BGR888p);
    input->setData(std::move(planar));
    return input;
}

void logPalmInference(
        const dai::NNData& inference,
    int64_t latestColorSequence,
        const std::vector<float>& scores,
        const std::vector<float>& regressors,
        std::size_t decodedCount,
        std::size_t suppressedCount) {
    static const bool enabled = std::getenv("SAI_HAND_TRACKING_DEBUG") != nullptr;
    static std::size_t inferenceCount = 0;
    if (!enabled || inferenceCount++ % 30 != 0) return;

    const float maxLogit = scores.empty() ? -std::numeric_limits<float>::infinity() :
        *std::max_element(scores.begin(), scores.end());
    const float maxProbability = 1.0f / (1.0f + std::exp(-maxLogit));
    std::clog << "[sai-hand] palm seq=" << inference.getSequenceNum()
        << " latest_color_seq=" << latestColorSequence
        << " frame_lag=" << latestColorSequence - inference.getSequenceNum()
        << " score_values=" << scores.size()
        << " regressor_values=" << regressors.size()
        << " max_logit=" << maxLogit
        << " max_probability=" << maxProbability
        << " decoded=" << decodedCount
        << " after_nms=" << suppressedCount << std::endl;
}

void logLandmarkInference(
        const dai::NNData& inference,
        const std::vector<float>& score,
        const std::vector<float>& handedness,
        const std::vector<float>& landmarks,
        const std::vector<float>& worldLandmarks) {
    static const bool enabled = std::getenv("SAI_HAND_TRACKING_DEBUG") != nullptr;
    static std::size_t inferenceCount = 0;
    if (!enabled || inferenceCount++ % 10 != 0) return;

    std::clog << "[sai-hand] landmark seq=" << inference.getSequenceNum()
        << " score_values=" << score.size()
        << " score=" << (score.empty() ? -1.0f : score[0])
        << " handedness_values=" << handedness.size()
        << " landmark_values=" << landmarks.size()
        << " world_landmark_values=" << worldLandmarks.size() << std::endl;
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
        callback_t_mapper_output onMapperOutput,
        char* errorMsg) {
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
        if (configuration->useColor || configuration->enableHandTracking) {
            handle->color->setInterleaved(false);
            handle->hooks.color = [colorFrames](std::shared_ptr<dai::ImgFrame> frame) {
                colorFrames->push(frame);
            };

            auto colorOutputNode = pipeline->create<dai::node::XLinkOut>();
            colorOutputNode->setStreamName("sai_color");
            colorOutputNode->input.setBlocking(false);
            colorOutputNode->input.setQueueSize(1);
            handle->color->preview.link(colorOutputNode->input);
        }

        bool handTrackingPipelineEnabled = configuration->enableHandTracking &&
            configuration->handTrackingPalmModelPath != nullptr &&
            configuration->handTrackingPalmModelPath[0] != '\0';
        bool handLandmarkPipelineEnabled = handTrackingPipelineEnabled &&
            configuration->handTrackingLandmarkModelPath != nullptr &&
            configuration->handTrackingLandmarkModelPath[0] != '\0';
        if (handTrackingPipelineEnabled) {
            auto palmFrameInput = pipeline->create<dai::node::XLinkIn>();
            palmFrameInput->setStreamName("sai_hand_palm_input");

            auto palmNetwork = pipeline->create<dai::node::NeuralNetwork>();
            palmNetwork->setBlobPath(configuration->handTrackingPalmModelPath);
            palmNetwork->input.setQueueSize(1);
            palmNetwork->input.setBlocking(false);
            palmFrameInput->out.link(palmNetwork->input);

            auto palmOutput = pipeline->create<dai::node::XLinkOut>();
            palmOutput->setStreamName("sai_hand_palm");
            palmOutput->input.setBlocking(false);
            palmOutput->input.setQueueSize(1);
            palmNetwork->out.link(palmOutput->input);
        }

        if (handLandmarkPipelineEnabled) {
            auto landmarkInput = pipeline->create<dai::node::XLinkIn>();
            landmarkInput->setStreamName("sai_hand_landmark_input");
            auto landmarkNetwork = pipeline->create<dai::node::NeuralNetwork>();
            landmarkNetwork->setBlobPath(configuration->handTrackingLandmarkModelPath);
            landmarkInput->out.link(landmarkNetwork->input);

            auto landmarkOutput = pipeline->create<dai::node::XLinkOut>();
            landmarkOutput->setStreamName("sai_hand_landmark");
            landmarkOutput->input.setBlocking(false);
            landmarkOutput->input.setQueueSize(4);
            landmarkNetwork->out.link(landmarkOutput->input);
        }

        std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>(*pipeline);
        std::shared_ptr<dai::DataOutputQueue> colorOutput =
            (configuration->useColor || configuration->enableHandTracking) ?
            device->getOutputQueue("sai_color", 1, false) : nullptr;
        std::shared_ptr<dai::DataOutputQueue> handTrackingOutput = handTrackingPipelineEnabled ?
            device->getOutputQueue("sai_hand_palm", 1, false) : nullptr;
        std::shared_ptr<dai::DataInputQueue> handTrackingPalmFrameInput = handTrackingPipelineEnabled ?
            device->getInputQueue("sai_hand_palm_input", 4, false) : nullptr;
        std::shared_ptr<dai::DataInputQueue> handTrackingLandmarkFrameInput = handLandmarkPipelineEnabled ?
            device->getInputQueue("sai_hand_landmark_input", 4, false) : nullptr;
        std::shared_ptr<dai::DataOutputQueue> handTrackingLandmarkOutput = handLandmarkPipelineEnabled ?
            device->getOutputQueue("sai_hand_landmark", 4, false) : nullptr;
        return new PipelineWrapper(
            handle, pipeline, device, colorFrames, colorOutput, handTrackingOutput,
            handTrackingPalmFrameInput, nullptr, handTrackingLandmarkFrameInput, handTrackingLandmarkOutput);
    } catch (const std::exception &e) {
        if (errorMsg != nullptr) {
            strncpy(errorMsg, e.what(), 1000 - 1);
            errorMsg[1000 - 1] = '\0';
        }
    } catch (...) {
        if (errorMsg != nullptr) {
            strncpy(errorMsg, "Unknown native exception while building the DepthAI pipeline.", 1000 - 1);
            errorMsg[1000 - 1] = '\0';
        }
    }

    return nullptr;
}

SessionWrapper* sai_depthai_pipeline_start_session(PipelineWrapper* pipelineHandle, char* errorMsg) {
    assert(pipelineHandle);
    try {
        return new SessionWrapper(
            pipelineHandle->getHandle()->startSession(*pipelineHandle->getDevice()),
            pipelineHandle->getColorFrames(),
            pipelineHandle->getColorOutput(),
            pipelineHandle->getHandTrackingOutput(),
            pipelineHandle->getHandTrackingPalmFrameInput(),
            pipelineHandle->getHandTrackingLandmarkConfig(),
            pipelineHandle->getHandTrackingLandmarkFrameInput(),
            pipelineHandle->getHandTrackingLandmarkOutput());
    } catch(const std::exception &e) {
        if (errorMsg != nullptr) {
            strncpy(errorMsg, e.what(), 1000 - 1);
            errorMsg[1000 - 1] = '\0'; // Ensure null-termination
        } else {
            return nullptr;
        }
    } catch (...) {
        if (errorMsg != nullptr) {
            strncpy(errorMsg, "Unknown native exception while starting the DepthAI session.", 1000 - 1);
            errorMsg[1000 - 1] = '\0';
        }
        return nullptr;
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
    if (const auto colorOutput = sessionHandle->getColorOutput()) {
        std::shared_ptr<dai::ImgFrame> frame;
        while (auto nextFrame = colorOutput->tryGet<dai::ImgFrame>()) {
            frame = nextFrame;
        }
        if (frame) sessionHandle->getColorFrames()->push(frame);
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

HandTrackingOutputWrapper* sai_depthai_session_get_hand_tracking_output(
        const SessionWrapper* sessionHandle) {
    assert(sessionHandle);
    const auto outputQueue = sessionHandle->getHandTrackingOutput();
    if (!outputQueue) return nullptr;

    const auto colorFrame = sessionHandle->getColorFrames()->getLatest();
    const auto palmFrameInputQueue = sessionHandle->getHandTrackingPalmFrameInput();
    auto* mutableSession = const_cast<SessionWrapper*>(sessionHandle);
    if (colorFrame && palmFrameInputQueue &&
        colorFrame->getSequenceNum() != mutableSession->getLastHandTrackingPalmConfigSequence()) {
        palmFrameInputQueue->send(createPalmInput(colorFrame));
        mutableSession->setLastHandTrackingPalmConfigSequence(colorFrame->getSequenceNum());
    }

    std::shared_ptr<dai::NNData> inference;
    while (auto nextInference = outputQueue->tryGet<dai::NNData>()) {
        inference = nextInference;
    }
    const auto landmarkFrameInputQueue = sessionHandle->getHandTrackingLandmarkFrameInput();
    const auto landmarkOutputQueue = sessionHandle->getHandTrackingLandmarkOutput();

    if (inference && !landmarkOutputQueue) {
        std::vector<float> scores = inference->getLayerFp16("classificators");
        std::vector<float> regressors = inference->getLayerFp16("regressors");
        if (scores.empty() || regressors.empty()) return nullptr;
        std::vector<saiHandTracking::PalmDetection> detections =
            saiHandTracking::decodePalmDetections(scores, regressors, 0.5f, false);
        const std::size_t decodedCount = detections.size();
        detections = saiHandTracking::suppressPalmDetections(std::move(detections), 0.3f, 2);
        logPalmInference(
            *inference,
            colorFrame ? colorFrame->getSequenceNum() : -1,
            scores,
            regressors,
            decodedCount,
            detections.size());
        mutableSession->setLatestHandTrackingOutput(
            std::move(detections),
            inference->getSequenceNum(),
            std::chrono::duration<double>(inference->getTimestampDevice().time_since_epoch()).count());
        return new HandTrackingOutputWrapper(
            mutableSession->getLatestHandTrackingDetections(),
            mutableSession->getLatestHandTrackingSequenceNumber(),
            mutableSession->getLatestHandTrackingTimestamp());
    }

    const auto palmSourceFrame = inference ?
        sessionHandle->getColorFrames()->getBySequence(inference->getSequenceNum()) : nullptr;
    const auto landmarkSourceFrame = palmSourceFrame ? palmSourceFrame : colorFrame;
    const float imageWidth = landmarkSourceFrame ?
        static_cast<float>(landmarkSourceFrame->getWidth()) : 640.0f;
    const float imageHeight = landmarkSourceFrame ?
        static_cast<float>(landmarkSourceFrame->getHeight()) : 360.0f;
    auto& pending = mutableSession->getPendingHandTrackingOutputs();
    if (inference) {
        std::vector<float> scores = inference->getLayerFp16("classificators");
        std::vector<float> regressors = inference->getLayerFp16("regressors");
        if (scores.empty() || regressors.empty()) return nullptr;
        std::vector<saiHandTracking::PalmDetection> detections =
            saiHandTracking::decodePalmDetections(scores, regressors, 0.5f, false);
        const std::size_t decodedCount = detections.size();
        detections = saiHandTracking::suppressPalmDetections(std::move(detections), 0.3f, 2);
        logPalmInference(
            *inference,
            colorFrame ? colorFrame->getSequenceNum() : -1,
            scores,
            regressors,
            decodedCount,
            detections.size());
        for (const auto& detection : detections) {
            if (landmarkSourceFrame && landmarkFrameInputQueue) {
                landmarkFrameInputQueue->send(createLandmarkInput(
                    landmarkSourceFrame, detection, inference->getSequenceNum()));
            }
        }

        pending.push_back({
            std::move(detections),
            inference->getSequenceNum(),
            std::chrono::duration<double>(
                inference->getTimestampDevice().time_since_epoch()).count(),
            0});
        while (pending.size() > 4) pending.pop_front();
    }

    while (auto landmarkInference = landmarkOutputQueue->tryGet<dai::NNData>()) {
        while (!pending.empty() &&
               pending.front().nextLandmarkIndex >= pending.front().detections.size()) {
            pending.pop_front();
        }
        if (pending.empty()) break;

        auto& frame = pending.front();
        auto& detection = frame.detections[frame.nextLandmarkIndex++];
        const std::vector<float> score = landmarkInference->hasLayer("Identity_1") ?
            landmarkInference->getLayerFp16("Identity_1") : std::vector<float>();
        const std::vector<float> handedness = landmarkInference->hasLayer("Identity_2") ?
            landmarkInference->getLayerFp16("Identity_2") : std::vector<float>();
        const std::vector<float> landmarks = landmarkInference->hasLayer("Identity_dense/BiasAdd/Add") ?
            landmarkInference->getLayerFp16("Identity_dense/BiasAdd/Add") : std::vector<float>();
        const std::vector<float> worldLandmarks = landmarkInference->hasLayer("Identity_3_dense/BiasAdd/Add") ?
            landmarkInference->getLayerFp16("Identity_3_dense/BiasAdd/Add") : std::vector<float>();
        logLandmarkInference(*landmarkInference, score, handedness, landmarks, worldLandmarks);
        const saiHandTracking::LandmarkRoi roi =
            saiHandTracking::calculateLandmarkRoi(detection, imageWidth, imageHeight);
        saiHandTracking::decodeLandmark(
            detection, score, handedness, landmarks, worldLandmarks, roi, imageWidth, imageHeight);
    }

    while (!pending.empty() && pending.front().nextLandmarkIndex >= pending.front().detections.size()) {
        PendingHandTrackingOutput completed = std::move(pending.front());
        pending.pop_front();
        completed.detections.erase(
            std::remove_if(
                completed.detections.begin(),
                completed.detections.end(),
                [](const saiHandTracking::PalmDetection& detection) {
                    return !detection.hasLandmarks;
                }),
            completed.detections.end());
        mutableSession->setLatestHandTrackingOutput(
            completed.detections,
            completed.sequenceNumber,
            completed.timestamp);
        return new HandTrackingOutputWrapper(
            std::move(completed.detections), completed.sequenceNumber, completed.timestamp);
    }

    if (!mutableSession->hasLatestHandTrackingOutput()) return nullptr;
    return new HandTrackingOutputWrapper(
        mutableSession->getLatestHandTrackingDetections(),
        mutableSession->getLatestHandTrackingSequenceNumber(),
        mutableSession->getLatestHandTrackingTimestamp());
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

float sai_hand_tracking_output_get_keypoint_value(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex,
        int valueIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).keypoints.at(valueIndex);
}

float sai_hand_tracking_output_get_landmark_score(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).landmarkScore;
}

float sai_hand_tracking_output_get_handedness(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).handedness;
}

int sai_hand_tracking_output_get_gesture(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).gesture;
}

float sai_hand_tracking_output_get_landmark_value(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex,
        int valueIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).landmarks.at(valueIndex);
}

float sai_hand_tracking_output_get_world_landmark_value(
        const HandTrackingOutputWrapper* outputHandle,
        int detectionIndex,
        int valueIndex) {
    assert(outputHandle);
    return outputHandle->getDetections().at(detectionIndex).worldLandmarks.at(valueIndex);
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
