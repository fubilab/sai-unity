#include "../include/spectacularAI/unity/depthai.hpp"

#include <spectacularAI/output.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

#ifdef _WIN32
class NativePreviewWindow {
public:
    bool create(int width, int height) {
        _className = "SpectacularAIHandPreview";
        WNDCLASSA windowClass{};
        windowClass.lpfnWndProc = &NativePreviewWindow::windowProcedure;
        windowClass.hInstance = GetModuleHandleA(nullptr);
        windowClass.lpszClassName = _className.c_str();
        windowClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
        windowClass.hbrBackground = static_cast<HBRUSH>(GetStockObject(BLACK_BRUSH));
        RegisterClassA(&windowClass);

        _window = CreateWindowA(
            _className.c_str(),
            "SpectacularAI native hand tracking preview",
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            std::max(640, width),
            std::max(480, height),
            nullptr,
            nullptr,
            windowClass.hInstance,
            this);
        if (_window == nullptr) return false;

        ShowWindow(_window, SW_SHOW);
        UpdateWindow(_window);
        return true;
    }

    bool pumpMessages() {
        MSG message{};
        while (PeekMessageA(&message, nullptr, 0, 0, PM_REMOVE)) {
            if (message.message == WM_QUIT) {
                return false;
            }
            TranslateMessage(&message);
            DispatchMessageA(&message);
        }
        return _window != nullptr;
    }

    void update(const std::vector<std::uint8_t>& pixels, int width, int height) {
        if (_window == nullptr) return;
        _pixels = pixels;
        _width = width;
        _height = height;
        InvalidateRect(_window, nullptr, FALSE);
        UpdateWindow(_window);
    }

    void setStatus(const std::string& status) {
        if (_window != nullptr) SetWindowTextA(_window, status.c_str());
    }

    ~NativePreviewWindow() {
        if (_window != nullptr) DestroyWindow(_window);
        if (!_className.empty()) UnregisterClassA(_className.c_str(), GetModuleHandleA(nullptr));
    }

private:
    static LRESULT CALLBACK windowProcedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
        NativePreviewWindow* preview = reinterpret_cast<NativePreviewWindow*>(
            GetWindowLongPtrA(window, GWLP_USERDATA));
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTA*>(lParam);
            preview = static_cast<NativePreviewWindow*>(create->lpCreateParams);
            SetWindowLongPtrA(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(preview));
            preview->_window = window;
        }

        if (preview != nullptr) {
            switch (message) {
            case WM_KEYDOWN:
                if (wParam == VK_ESCAPE || wParam == 'Q') {
                    DestroyWindow(window);
                    return 0;
                }
                break;
            case WM_CLOSE:
                DestroyWindow(window);
                return 0;
            case WM_DESTROY:
                preview->_window = nullptr;
                PostQuitMessage(0);
                return 0;
            case WM_PAINT:
                preview->paint();
                return 0;
            default:
                break;
            }
        }
        return DefWindowProcA(window, message, wParam, lParam);
    }

    void paint() {
        PAINTSTRUCT paintStruct{};
        HDC context = BeginPaint(_window, &paintStruct);
        RECT clientRect{};
        GetClientRect(_window, &clientRect);
        if (!_pixels.empty() && _width > 0 && _height > 0) {
            const int clientWidth = clientRect.right - clientRect.left;
            const int clientHeight = clientRect.bottom - clientRect.top;
            const float scale = std::min(
                static_cast<float>(clientWidth) / _width,
                static_cast<float>(clientHeight) / _height);
            const int drawWidth = static_cast<int>(std::lround(_width * scale));
            const int drawHeight = static_cast<int>(std::lround(_height * scale));
            const int drawX = (clientWidth - drawWidth) / 2;
            const int drawY = (clientHeight - drawHeight) / 2;
            BITMAPINFO bitmapInfo{};
            bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
            bitmapInfo.bmiHeader.biWidth = _width;
            bitmapInfo.bmiHeader.biHeight = -_height;
            bitmapInfo.bmiHeader.biPlanes = 1;
            bitmapInfo.bmiHeader.biBitCount = 32;
            bitmapInfo.bmiHeader.biCompression = BI_RGB;
            StretchDIBits(
                context,
                drawX,
                drawY,
                drawWidth,
                drawHeight,
                0,
                0,
                _width,
                _height,
                _pixels.data(),
                &bitmapInfo,
                DIB_RGB_COLORS,
                SRCCOPY);
        }
        EndPaint(_window, &paintStruct);
    }

    HWND _window = nullptr;
    std::string _className;
    std::vector<std::uint8_t> _pixels;
    int _width = 0;
    int _height = 0;
};
#endif

void setPixel(
        std::vector<std::uint8_t>& pixels,
        int width,
        int height,
        int x,
        int y,
        std::uint8_t blue,
        std::uint8_t green,
        std::uint8_t red) {
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    const std::size_t index = static_cast<std::size_t>(y * width + x) * 4;
    pixels[index] = blue;
    pixels[index + 1] = green;
    pixels[index + 2] = red;
    pixels[index + 3] = 255;
}

void drawLine(
        std::vector<std::uint8_t>& pixels,
        int width,
        int height,
        int x0,
        int y0,
        int x1,
        int y1,
        std::uint8_t blue,
        std::uint8_t green,
        std::uint8_t red,
        int thickness = 2) {
    const int distance = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
    for (int step = 0; step <= distance; ++step) {
        const float t = distance == 0 ? 0.0f : static_cast<float>(step) / distance;
        const int x = static_cast<int>(std::lround(x0 + (x1 - x0) * t));
        const int y = static_cast<int>(std::lround(y0 + (y1 - y0) * t));
        for (int offset = -thickness / 2; offset <= thickness / 2; ++offset) {
            setPixel(pixels, width, height, x + offset, y, blue, green, red);
            setPixel(pixels, width, height, x, y + offset, blue, green, red);
        }
    }
}

void drawCircle(
        std::vector<std::uint8_t>& pixels,
        int width,
        int height,
        int centerX,
        int centerY,
        int radius,
        std::uint8_t blue,
        std::uint8_t green,
        std::uint8_t red) {
    for (int y = -radius; y <= radius; ++y) {
        for (int x = -radius; x <= radius; ++x) {
            if (x * x + y * y <= radius * radius) {
                setPixel(pixels, width, height, centerX + x, centerY + y, blue, green, red);
            }
        }
    }
}

int imageCoordinate(float normalizedCoordinate, int imageSize) {
    return static_cast<int>(std::lround(normalizedCoordinate * imageSize));
}

struct PreviewHand {
    float landmarkScore = 0.0f;
    float handedness = 0.5f;
    int gesture = 0;
    float stereoDepth = 0.0f;
    float handScale = 1.0f;
    bool placed = false;
    std::array<float, 63> image{};
    std::array<float, 63> camera{};
};

void drawHand(
        std::vector<std::uint8_t>& pixels,
        int width,
        int height,
        const PreviewHand& hand) {
    constexpr int handLines[][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {0, 5}, {5, 6}, {6, 7}, {7, 8},
        {5, 9}, {9, 10}, {10, 11}, {11, 12}, {9, 13}, {13, 14}, {14, 15}, {15, 16},
        {13, 17}, {17, 18}, {18, 19}, {19, 20}, {0, 17}};
    // right hand green, left hand orange (BGR)
    const std::uint8_t blue = 0;
    const std::uint8_t green = hand.handedness > 0.5f ? 255 : 165;
    const std::uint8_t red = hand.handedness > 0.5f ? 0 : 255;
    for (const auto& line : handLines) {
        const int start = line[0] * 3;
        const int end = line[1] * 3;
        drawLine(
            pixels, width, height,
            imageCoordinate(hand.image[start], width), imageCoordinate(hand.image[start + 1], height),
            imageCoordinate(hand.image[end], width), imageCoordinate(hand.image[end + 1], height),
            blue, green, red, 2);
    }
    for (int landmarkIndex = 0; landmarkIndex < 21; ++landmarkIndex) {
        const int valueIndex = landmarkIndex * 3;
        drawCircle(
            pixels, width, height,
            imageCoordinate(hand.image[valueIndex], width), imageCoordinate(hand.image[valueIndex + 1], height),
            3, 0, 0, 255);
    }
}

void convertPlanarRgbToBgra(
        const std::uint8_t* planar,
        int width,
        int height,
        std::vector<std::uint8_t>& pixels) {
    const std::size_t pixelCount = static_cast<std::size_t>(width) * height;
    pixels.resize(pixelCount * 4);
    for (std::size_t index = 0; index < pixelCount; ++index) {
        pixels[index * 4] = planar[pixelCount * 2 + index];
        pixels[index * 4 + 1] = planar[pixelCount + index];
        pixels[index * 4 + 2] = planar[index];
        pixels[index * 4 + 3] = 255;
    }
}

std::string gestureName(int gesture) {
    switch (gesture) {
    case 1: return "ONE";
    case 2: return "TWO";
    case 3: return "THREE";
    case 4: return "FOUR";
    case 5: return "FIVE";
    case 6: return "FIST";
    case 7: return "OK";
    case 8: return "PEACE";
    default: return "-";
    }
}

}

int main(int argc, char *argv[]) {
    // usage: main_depthai [palm_detection.blob hand_landmark.blob]
    // environment: SAI_RECORD_DIR, SAI_RUN_SECONDS, SAI_IR_PROJECTOR (0..1), SAI_HAND_NO_DEPTH,
    //              SAI_HAND_REPLAY (color.raw), SAI_HAND_LOG (jsonl), SAI_HAND_TRACKING_DEBUG
    ConfigurationWrapper config;
    config.lowLatency = true;
    config.useColor = true;
    if (argc >= 3) {
        config.enableHandTracking = true;
        config.handTrackingPalmModelPath = argv[1];
        config.handTrackingLandmarkModelPath = argv[2];
    }
    if (const char* ir = std::getenv("SAI_IR_PROJECTOR")) config.handTrackingIrProjectorIntensity = static_cast<float>(std::atof(ir));
    if (std::getenv("SAI_HAND_NO_DEPTH")) config.handTrackingUseDepth = false;
    const bool replaying = std::getenv("SAI_HAND_REPLAY") != nullptr;
    // Diagnostics: toggle SAI components to see how they affect the hand tracking networks
    if (const char* v = std::getenv("SAI_USE_COLOR")) config.useColor = std::atoi(v) != 0;
    if (const char* v = std::getenv("SAI_USE_STEREO")) config.useStereo = std::atoi(v) != 0;
    if (const char* v = std::getenv("SAI_USE_FEATURE_TRACKER")) config.useFeatureTracker = std::atoi(v) != 0;
    if (const char* v = std::getenv("SAI_FAST_VIO")) config.fastVio = std::atoi(v) != 0;
    if (const char* v = std::getenv("SAI_LOW_LATENCY")) config.lowLatency = std::atoi(v) != 0;

    // SLAM callback
    callback_t_mapper_output onMapperOutput = [](const MapperOutputWrapper* mapperOutput) {
        const int64_t* updatedKeyFrames;
        int32_t nUpdatedKeyFrames = sai_mapper_output_get_updated_key_frames(mapperOutput, &updatedKeyFrames);
        if (sai_mapper_output_get_final_map(mapperOutput)) {
            std::cout << "SLAM: final map: " << nUpdatedKeyFrames << std::endl;
        } else {
            std::cout << "SLAM: update map: " << nUpdatedKeyFrames << std::endl;
        }
        sai_mapper_output_release(mapperOutput); // must release memory!
    };

    // Optional VIO internal parameter: area of the image that the feature tracker ignores (the projector image and the
    // hands on the ARML), x0,y0,x1,y1 in 0..1. Same as 'trackerMasks' of the launcher settings.
    std::vector<std::string> internalParameterStrings;
    if (const char* masks = std::getenv("SAI_VIO_TRACKER_MASKS")) {
        internalParameterStrings = {"trackerMasks", masks};
    }
    std::vector<const char*> internalParameters;
    for (const std::string& s : internalParameterStrings) internalParameters.push_back(s.c_str());

    std::array<char, 1000> errorMessage{};
    PipelineWrapper* pipeline = sai_depthai_pipeline_build(
        &config, internalParameters.empty() ? nullptr : internalParameters.data(),
        static_cast<int>(internalParameters.size() / 2),
        // like the Unity wrapper, which has the mapping API off by default (SAI_MAPPING=1 turns it on)
        std::getenv("SAI_MAPPING") ? onMapperOutput : nullptr);
    if (!pipeline) {
        sai_depthai_get_last_error(errorMessage.data(), static_cast<int>(errorMessage.size()));
        std::cerr << "Pipeline build failed: " << errorMessage.data() << std::endl;
        return 1;
    }

    errorMessage.fill('\0');
    SessionWrapper* session = sai_depthai_pipeline_start_session(
        pipeline, errorMessage.data());
    if (!session) {
        std::cerr << "Session start failed: " << errorMessage.data() << std::endl;
        sai_depthai_pipeline_release(pipeline);
        return 1;
    }

    // Optional raw recorder (SAI_RECORD_DIR): exactly the color frames the hand pipeline sees,
    // plus VIO poses and RGB camera intrinsics, so they can be replayed offline (SAI_HAND_REPLAY).
    const char* recordDirEnv = std::getenv("SAI_RECORD_DIR");
    std::ofstream recordColor, recordVio;
    bool recordIntrinsicsWritten = false;
    const auto recordStart = std::chrono::steady_clock::now();
    const auto hostSeconds = [recordStart]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - recordStart).count();
    };
    if (recordDirEnv != nullptr && recordDirEnv[0] != '\0') {
        const std::string recordDir = recordDirEnv;
        if (!std::getenv("SAI_RECORD_NO_COLOR")) recordColor.open(recordDir + "/color.raw", std::ios::binary);
        recordVio.open(recordDir + "/vio.jsonl");
        std::cout << "recording to " << recordDir << std::endl;
    }
    int runSeconds = replaying ? 3600 : 30;
    if (const char* runSecondsEnv = std::getenv("SAI_RUN_SECONDS")) runSeconds = std::max(1, std::atoi(runSecondsEnv));

    int counter = 0;
    int64_t lastColorSequence = -1;
    int64_t lastHandSequence = -1;
    const int handStride = sai_hand_tracking_float_stride();
    std::vector<float> handBuffer(static_cast<std::size_t>(handStride) * 2);
    std::vector<PreviewHand> latestHands;
    double latestHandLatency = 0.0;
    std::vector<std::uint8_t> previewPixels;
#ifdef _WIN32
    NativePreviewWindow previewWindow;
    bool previewCreated = false;
#endif
    auto nextStatusTime = std::chrono::steady_clock::now();
    const auto deadline = nextStatusTime + std::chrono::seconds(runSeconds);
    int handFrames = 0;
    auto nextDeviceStats = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
        int colorWidth = 0;
        int colorHeight = 0;
        bool hasColorFrame = false;
        int64_t colorSequence = -1;
        ColorFrameWrapper* colorFrame = sai_depthai_session_get_color_frame(session);
        if (colorFrame != nullptr) {
            colorSequence = sai_color_frame_get_sequence_number(colorFrame);
            colorWidth = static_cast<int>(sai_color_frame_get_width(colorFrame));
            colorHeight = static_cast<int>(sai_color_frame_get_height(colorFrame));
            const std::size_t pixelCount = static_cast<std::size_t>(colorWidth) * colorHeight;
            const std::uint8_t* data = sai_color_frame_get_data(colorFrame);
            const unsigned int dataSize = sai_color_frame_get_data_size(colorFrame);
            if (data != nullptr && dataSize >= pixelCount * 3 && colorWidth > 0 && colorHeight > 0) {
                convertPlanarRgbToBgra(data, colorWidth, colorHeight, previewPixels);
                hasColorFrame = true;
            }
            if (colorSequence != lastColorSequence) {
                lastColorSequence = colorSequence;
                if (recordColor.is_open() && data != nullptr && dataSize >= pixelCount * 3) {
                    struct RecordHeader {
                        int64_t sequence;
                        double deviceTimestamp;
                        double hostTime;
                        int32_t width;
                        int32_t height;
                        int32_t dataSize;
                        int32_t reserved;
                    } header{
                        colorSequence,
                        sai_color_frame_get_timestamp(colorFrame),
                        hostSeconds(),
                        colorWidth,
                        colorHeight,
                        static_cast<int32_t>(pixelCount * 3),
                        0};
                    recordColor.write(reinterpret_cast<const char*>(&header), sizeof(header));
                    recordColor.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(pixelCount * 3));
                }
            }
            sai_color_frame_release(colorFrame);
        }

        if (config.enableHandTracking) {
            int64_t handSequence = -1;
            double handTimestamp = 0.0;
            double handLatency = 0.0;
            const int handCount = sai_depthai_session_get_hand_frame(
                session, lastHandSequence, &handSequence, &handTimestamp, &handLatency,
                handBuffer.data(), static_cast<int>(handBuffer.size()));
            if (handCount >= 0) {
                lastHandSequence = handSequence;
                latestHandLatency = handLatency;
                ++handFrames;
                latestHands.clear();
                for (int index = 0; index < handCount; ++index) {
                    const float* block = handBuffer.data() + static_cast<std::size_t>(index) * handStride;
                    PreviewHand hand;
                    hand.landmarkScore = block[1];
                    hand.handedness = block[2];
                    hand.gesture = static_cast<int>(block[3]);
                    hand.handScale = block[5];
                    hand.stereoDepth = block[6];
                    hand.placed = block[7] > 0.5f;
                    std::copy(block + 8, block + 8 + 63, hand.image.begin());
                    std::copy(block + 8 + 63, block + 8 + 126, hand.camera.begin());
                    latestHands.push_back(hand);
                }
            }
            if (std::chrono::steady_clock::now() >= nextStatusTime) {
                std::cout << "hands: " << latestHands.size() << " latency " << latestHandLatency << " ms, "
                    << handFrames << " results/s";
                for (const PreviewHand& hand : latestHands) {
                    std::cout << " | " << (hand.handedness > 0.5f ? "R" : "L") << " " << gestureName(hand.gesture)
                        << " lm=" << hand.landmarkScore << " wrist(cam)=" << hand.camera[0] << ","
                        << hand.camera[1] << "," << hand.camera[2] << " stereo=" << hand.stereoDepth
                        << " scale=" << hand.handScale;
                }
                std::cout << std::endl;
                handFrames = 0;
                nextStatusTime = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            }
            if (replaying && sai_depthai_session_hand_replay_finished(session)) break;
        }

#ifdef _WIN32
        if (hasColorFrame && !replaying) {
            if (!previewCreated) {
                previewCreated = previewWindow.create(colorWidth, colorHeight);
                if (!previewCreated) {
                    std::cerr << "Could not create native preview window" << std::endl;
                }
            }
            if (previewCreated) {
                for (const PreviewHand& hand : latestHands) {
                    drawHand(previewPixels, colorWidth, colorHeight, hand);
                }
                std::ostringstream title;
                title << "SpectacularAI native hand tracking - frame=" << colorSequence
                    << " hands=" << latestHands.size() << " latency=" << static_cast<int>(latestHandLatency) << "ms";
                for (const PreviewHand& hand : latestHands) {
                    title << " | " << (hand.handedness > 0.5f ? "R" : "L") << " " << gestureName(hand.gesture)
                        << " z=" << hand.camera[2] << "m";
                }
                previewWindow.setStatus(title.str());
                previewWindow.update(previewPixels, colorWidth, colorHeight);
            }
        }
        if (previewCreated && !previewWindow.pumpMessages()) break;
#else
        (void)hasColorFrame;
#endif

        if (sai_depthai_session_has_output(session)) {
            ++counter;
            VioOutputWrapper* output = sai_depthai_session_get_output(session);
            spectacularAI::Pose pose = sai_vio_output_get_pose(output);
            if (recordVio.is_open()) {
                recordVio << "{\"t\":" << pose.time << ",\"host\":" << hostSeconds()
                    << ",\"status\":" << static_cast<int>(sai_vio_output_get_tracking_status(output))
                    << ",\"p\":[" << pose.position.x << ',' << pose.position.y << ',' << pose.position.z
                    << "],\"q\":[" << pose.orientation.x << ',' << pose.orientation.y << ','
                    << pose.orientation.z << ',' << pose.orientation.w << "]";
                spectacularAI::CameraPose* rgbPose = sai_depthai_session_get_rgb_camera_pose(session, output);
                if (rgbPose != nullptr) {
                    const Matrix4dWrapper m = sai_camera_pose_get_camera_to_world_matrix(rgbPose);
                    recordVio << ",\"rgb_c2w\":[" << m.m00 << ',' << m.m01 << ',' << m.m02 << ',' << m.m03 << ','
                        << m.m10 << ',' << m.m11 << ',' << m.m12 << ',' << m.m13 << ','
                        << m.m20 << ',' << m.m21 << ',' << m.m22 << ',' << m.m23 << ','
                        << m.m30 << ',' << m.m31 << ',' << m.m32 << ',' << m.m33 << "]";
                    if (!recordIntrinsicsWritten) {
                        if (const CameraWrapper* camera = sai_camera_pose_get_camera(rgbPose)) {
                            const Matrix3dWrapper k = sai_camera_get_intrinsic_matrix(camera);
                            recordVio << ",\"rgb_intrinsics\":[" << k.m00 << ',' << k.m01 << ',' << k.m02 << ','
                                << k.m10 << ',' << k.m11 << ',' << k.m12 << ','
                                << k.m20 << ',' << k.m21 << ',' << k.m22 << "]";
                            std::cout << "rgb intrinsics fx=" << k.m00 << " fy=" << k.m11
                                << " cx=" << k.m02 << " cy=" << k.m12 << std::endl;
                            recordIntrinsicsWritten = true;
                        }
                    }
                    sai_camera_pose_release(rgbPose);
                }
                recordVio << "}\n";
                recordVio.flush();
            }
            if (counter % 60 == 0) {
                std::cout << counter << ". position = " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
            }
            sai_vio_output_release(output); // must release memory!
        }

        if (std::getenv("SAI_HAND_TRACKING_DEBUG") && std::chrono::steady_clock::now() >= nextDeviceStats) {
            sai_depthai_pipeline_print_device_stats(pipeline);
            nextDeviceStats = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sai_depthai_session_release(session); // must release memory!
    sai_depthai_pipeline_release(pipeline); // must release memory!

    return 0;
}
