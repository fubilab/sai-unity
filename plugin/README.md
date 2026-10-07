# spectacularAI_unity plugin

## Dependencies

The `spectacularAI_depthaiPlugin` package is required. For non-commercial purposes you can find one here: https://github.com/SpectacularAI/sdk/releases

## Building (Windows)
```
mkdir target && cd target
cmake -Ddepthai_DIR=path\to\spectacularAI_depthaiPlugin_cpp_non-commercial_1.36.0\Windows\lib\cmake\depthai -DspectacularAI_depthaiPlugin_DIR=path\to\spectacularAI_depthaiPlugin_cpp_non-commercial_1.36.0\Windows\lib\cmake\spectacularAI ..
cmake --build . --config Release
```

Replace the existing `spectacularAI_unity.dll` [here](https://github.com/SpectacularAI/unity-wrapper/tree/main/unity-examples/Assets/SpectacularAI/Plugins/Windows).

## Building (Linux)
```
mkdir target && cd target
cmake -Ddepthai_DIR=path/to/spectacularAI_depthaiPlugin_cpp_non-commercial_1.36.0/Linux_Ubuntu_x86-64/lib/cmake/depthai -DspectacularAI_depthaiPlugin_DIR=path/to/spectacularAI_depthaiPlugin_cpp_non-commercial_1.36.0/Linux_Ubuntu_x86-64/lib/cmake/spectacularAI ..
make
```
Replace the existing `libspectacularAI_unity.so` [here](https://github.com/SpectacularAI/unity-wrapper/tree/main/unity-examples/Assets/SpectacularAI/Plugins/Linux_Ubuntu_x86-64).

## Run C++ examples (for debugging/testing)
1. Live example with DepthAI devices. Connect DepthAI device and then run
```
.\Release\main_depthai.exe path\to\palm_detection_sh4.blob path\to\hand_landmark_lite_sh4.blob
```
The terminal prints color-frame progress and native hand output. A line such as
`hand output 123 (0 detections)` means the native pipeline is running but no palm
was detected. A line such as `hand[0] palm=... landmark=...` confirms a decoded hand.

2. If you have recorded datasets, then you can replay them using
```
.\Release\main_replay.exe path\to\recording
```
The position of the device should be printed in your terminal.
