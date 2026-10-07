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

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

struct PreviewDetection {
    float score = 0.0f;
    float landmarkScore = 0.0f;
    float handedness = 0.5f;
    int gesture = -1;
    std::array<float, 4> box{};
    std::array<float, 14> keypoints{};
    std::array<float, 63> landmarks{};
    bool hasLandmarks = false;
};

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
            BITMAPINFO bitmapInfo{};
            bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
            bitmapInfo.bmiHeader.biWidth = _width;
            bitmapInfo.bmiHeader.biHeight = -_height;
            bitmapInfo.bmiHeader.biPlanes = 1;
            bitmapInfo.bmiHeader.biBitCount = 32;
            bitmapInfo.bmiHeader.biCompression = BI_RGB;
            StretchDIBits(
                context,
                0,
                0,
                clientRect.right,
                clientRect.bottom,
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

int squareCoordinate(float normalizedCoordinate, int imageWidth, int imageHeight, bool horizontal) {
    const int frameSize = std::max(imageWidth, imageHeight);
    const int padding = horizontal ? (frameSize - imageWidth) / 2 : (frameSize - imageHeight) / 2;
    return static_cast<int>(std::lround(normalizedCoordinate * frameSize - padding));
}

int imageCoordinate(float normalizedCoordinate, int imageSize) {
    return static_cast<int>(std::lround(normalizedCoordinate * imageSize));
}

void drawDetection(
        std::vector<std::uint8_t>& pixels,
        int width,
        int height,
        const PreviewDetection& detection) {
    const int left = squareCoordinate(detection.box[0], width, height, true);
    const int top = squareCoordinate(detection.box[1], width, height, false);
    const int right = squareCoordinate(detection.box[0] + detection.box[2], width, height, true);
    const int bottom = squareCoordinate(detection.box[1] + detection.box[3], width, height, false);
    for (int offset = 0; offset < 3; ++offset) {
        drawLine(pixels, width, height, left, top + offset, right, top + offset, 0, 255, 0);
        drawLine(pixels, width, height, left, bottom - offset, right, bottom - offset, 0, 255, 0);
        drawLine(pixels, width, height, left + offset, top, left + offset, bottom, 0, 255, 0);
        drawLine(pixels, width, height, right - offset, top, right - offset, bottom, 0, 255, 0);
    }

    for (int keypointIndex = 0; keypointIndex < 7; ++keypointIndex) {
        drawCircle(
            pixels,
            width,
            height,
            squareCoordinate(detection.keypoints[keypointIndex * 2], width, height, true),
            squareCoordinate(detection.keypoints[keypointIndex * 2 + 1], width, height, false),
            4,
            0,
            0,
            255);
    }

    if (!detection.hasLandmarks) return;
    constexpr int handLines[][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {0, 5}, {5, 6}, {6, 7}, {7, 8},
        {5, 9}, {9, 10}, {10, 11}, {11, 12}, {9, 13}, {13, 14}, {14, 15}, {15, 16},
        {13, 17}, {17, 18}, {18, 19}, {19, 20}, {0, 17}};
    for (const auto& line : handLines) {
        const int start = line[0] * 3;
        const int end = line[1] * 3;
        drawLine(
            pixels,
            width,
            height,
            imageCoordinate(detection.landmarks[start], width),
            imageCoordinate(detection.landmarks[start + 1], height),
            imageCoordinate(detection.landmarks[end], width),
            imageCoordinate(detection.landmarks[end + 1], height),
            255,
            220,
            0,
            3);
    }
    for (int landmarkIndex = 0; landmarkIndex < 21; ++landmarkIndex) {
        const int valueIndex = landmarkIndex * 3;
        drawCircle(
            pixels,
            width,
            height,
            imageCoordinate(detection.landmarks[valueIndex], width),
            imageCoordinate(detection.landmarks[valueIndex + 1], height),
            4,
            255,
            255,
            255);
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

PreviewDetection copyDetection(const HandTrackingOutputWrapper* output, int index) {
    PreviewDetection detection;
    detection.score = sai_hand_tracking_output_get_score(output, index);
    detection.landmarkScore = sai_hand_tracking_output_get_landmark_score(output, index);
    detection.handedness = sai_hand_tracking_output_get_handedness(output, index);
    detection.gesture = sai_hand_tracking_output_get_gesture(output, index);
    for (int valueIndex = 0; valueIndex < 4; ++valueIndex) {
        detection.box[valueIndex] = sai_hand_tracking_output_get_box_value(output, index, valueIndex);
    }
    for (int valueIndex = 0; valueIndex < 14; ++valueIndex) {
        detection.keypoints[valueIndex] = sai_hand_tracking_output_get_keypoint_value(output, index, valueIndex);
    }
    for (int valueIndex = 0; valueIndex < 63; ++valueIndex) {
        detection.landmarks[valueIndex] = sai_hand_tracking_output_get_landmark_value(output, index, valueIndex);
    }
    detection.hasLandmarks = detection.landmarkScore > 0.5f;
    return detection;
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
    default: return "UNKNOWN";
    }
}

}

int main(int argc, char *argv[]) {
    ConfigurationWrapper config;
    config.lowLatency = true;
    config.useColor = true;
    if (argc >= 3) {
        config.enableHandTracking = true;
        config.handTrackingPalmModelPath = argv[1];
        config.handTrackingLandmarkModelPath = argv[2];
    }

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

    std::array<char, 1000> errorMessage{};
    PipelineWrapper* pipeline = sai_depthai_pipeline_build(
        &config, nullptr, 0, onMapperOutput, errorMessage.data());
    if (!pipeline) {
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

    int counter = 0;
    int64_t lastColorSequence = -1;
    int64_t lastPalmSequence = -1;
    int handPollCount = 0;
    std::vector<PreviewDetection> latestDetections;
    std::vector<std::uint8_t> previewPixels;
#ifdef _WIN32
    NativePreviewWindow previewWindow;
    bool previewCreated = false;
#endif
    auto nextStatusTime = std::chrono::steady_clock::now();
    const auto deadline = nextStatusTime + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < deadline) {
        ++handPollCount;
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
                std::cout << "color frame " << colorSequence << " ("
                    << colorWidth << "x" << colorHeight << ")" << std::endl;
                lastColorSequence = colorSequence;
            }
            sai_color_frame_release(colorFrame);
        }

        HandTrackingOutputWrapper* handOutput =
            sai_depthai_session_get_hand_tracking_output(session);
        if (handOutput != nullptr) {
            int64_t sequence = sai_hand_tracking_output_get_sequence_number(handOutput);
            int detectionCount = sai_hand_tracking_output_get_count(handOutput);
            if (sequence != lastPalmSequence) {
                latestDetections.clear();
                std::cout << "hand output " << sequence << " ("
                    << detectionCount << " detections)" << std::endl;
                for (int detectionIndex = 0; detectionIndex < detectionCount; ++detectionIndex) {
                    latestDetections.push_back(copyDetection(handOutput, detectionIndex));
                    std::cout << "  hand[" << detectionIndex << "]"
                        << " palm="
                        << sai_hand_tracking_output_get_score(handOutput, detectionIndex)
                        << " landmark="
                        << sai_hand_tracking_output_get_landmark_score(handOutput, detectionIndex)
                        << " handedness="
                        << sai_hand_tracking_output_get_handedness(handOutput, detectionIndex)
                        << " gesture="
                        << sai_hand_tracking_output_get_gesture(handOutput, detectionIndex)
                        << std::endl;
                }
                lastPalmSequence = sequence;
            }
            if (std::chrono::steady_clock::now() >= nextStatusTime) {
                std::cout << "hand status: latest sequence=" << sequence
                    << ", detections=" << detectionCount << std::endl;
                nextStatusTime = std::chrono::steady_clock::now() + std::chrono::seconds(1);
            }
            sai_hand_tracking_output_release(handOutput);
        }
        else if (std::chrono::steady_clock::now() >= nextStatusTime) {
            std::cout << "hand status: no decoded output yet (poll "
                << handPollCount << ")" << std::endl;
            nextStatusTime = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }

#ifdef _WIN32
        if (hasColorFrame) {
            if (!previewCreated) {
                previewCreated = previewWindow.create(colorWidth, colorHeight);
                if (!previewCreated) {
                    std::cerr << "Could not create native preview window" << std::endl;
                }
            }
            if (previewCreated) {
                for (const PreviewDetection& detection : latestDetections) {
                    drawDetection(previewPixels, colorWidth, colorHeight, detection);
                }
                std::ostringstream title;
                title << "SpectacularAI native hand tracking preview - frame=" << colorSequence
                    << " hands=" << latestDetections.size();
                for (std::size_t index = 0; index < latestDetections.size(); ++index) {
                    const PreviewDetection& detection = latestDetections[index];
                    title << " | " << index << ": palm=" << detection.score
                        << " lm=" << detection.landmarkScore
                        << " " << (detection.handedness > 0.5f ? "right" : "left")
                        << " " << gestureName(detection.gesture);
                }
                previewWindow.setStatus(title.str());
                previewWindow.update(previewPixels, colorWidth, colorHeight);
            }
        }
        if (previewCreated && !previewWindow.pumpMessages()) break;
#endif

        if (sai_depthai_session_has_output(session)) {
            ++counter;
            VioOutputWrapper* output = sai_depthai_session_get_output(session);
            spectacularAI::Pose pose = sai_vio_output_get_pose(output);
            std::cout << counter << ". position = " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
            sai_vio_output_release(output); // must release memory!
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sai_depthai_session_release(session); // must release memory!
    sai_depthai_pipeline_release(pipeline); // must release memory!

    return 0;
}
