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
In arml-sdk it is `arml-unity/Assets/ARML/ARMLCore/Plugins/SpectacularAI/Plugins/Linux_Ubuntu_x86-64/`, next to the
1.39.0 `libspectacularAI_depthaiPlugin.so` and `libdepthai-core.so`: build against the same package version.

The hand tracking code compiles and its unit tests (`./main_hand_tracking`) pass on Linux with the 1.39.0 package
(GCC 13, C++14, Ubuntu 24.04 in WSL, no code changes needed), and the library exports every function the Unity C#
imports. libusb headers are not needed to build it. Build on Ubuntu 22.04, the OS of the ARML (the ARML itself can do
it): a library built on a newer Ubuntu links against a newer glibc (24.04: `GLIBC_2.38`) and does not load on 22.04.
Check with `objdump -T libspectacularAI_unity.so | grep -o 'GLIBC_[0-9.]*' | sort -uV | tail -3` (the stock one needs
2.32). The Unity C# also works with the old `.so`, only without hand tracking.

## Hand tracking

The plugin can run the MediaPipe hand models (palm detection + hand landmarks) on the same OAK-D as the VIO, in the
same DepthAI pipeline. The device can only be opened by one process, so this replaces a separate hand tracking
process. It is a port of the host mode of [depthai_hand_tracker](https://github.com/geaxgx/depthai_hand_tracker):
palm detection runs only when no hand is tracked, otherwise the ROI comes from the previous landmarks. A worker
thread feeds the networks, so Unity's main thread never blocks.

Enable with `ConfigurationWrapper.enableHandTracking` and the paths of `palm_detection_sh4.blob` and
`hand_landmark_full_sh4.blob` (`lite` is faster, less accurate). Models: `depthai_hand_tracker/models`.
Read results with `sai_depthai_session_get_hand_frame` (see `depthai.hpp`). Per hand it returns the landmarks in the
image, in meters in the camera frame, and hand centered. The metric position comes from fitting the model's metric
hand to the 2D landmarks. When `handTrackingUseDepth` is set, stereo depth at the palm calibrates the size of the
user's hand. The OAK-D used for the development (not the standard ARML setup) has an IR-pass lens on its right
camera, so there stereo only works with the IR dot projector (`handTrackingIrProjectorIntensity` > 0); with a
standard right camera try depth with the projector off first. Without depth the size of an average hand is assumed.

Notes
 * The networks share the device with the Spectacular AI pipeline, which keeps the device CPU busy: every network
   call takes about 50-65 ms (about 24 ms on a device running only the hand networks). `fastVio` brings this down.
 * The device heats up. In free air on a desk, Spectacular AI alone settles at about 78 C chip temperature and with
   hand tracking the device reached 100 C within four minutes, after which the networks stopped answering. To limit
   this, palm detection is only run every `searchIntervalMs` (100 ms) while no hand is tracked, and above
   `throttleStartC` (88 C) the work is reduced in three steps. `sai_depthai_session_get_hand_status` reports the
   temperature, the reduction level and whether the networks stalled. Measure the temperature in your enclosure.
 * The palm model only sees 128 x 128 pixels. In the whole 16:9 frame (letterboxed into a square) a hand at arm's
   length is only ~30 pixels there, and the model often misses it: on a screenshot from an ARML a hand held ~30 cm from
   the camera was not found at all (best palm score 0.59, on a poster on the wall, rejected by the landmark model).
   While no hand is tracked, each palm detection run therefore looks at one 360 x 360 crop of the frame and cycles
   through the center, the left, the right and the whole frame (`searchCrops`, `SAI_HAND_SEARCH_CROPS=0` turns it
   off). The same image is found at the first run with the center crop (palm score 0.73, landmark confidence 0.99).
   A hand is found within one cycle (4 runs, 100 ms apart while idle). Once found, the tracking from the previous
   landmarks uses the full image resolution, so it does not depend on this.
 * Spots of a scene (a poster, a shadow) can score above the palm threshold again and again (on the ARML about half of
   the idle palm runs found one, at a palm score of ~0.59, where real new hands score 0.77 on median) and every one
   costs a landmark run of 60-100 ms that rejects it. A candidate that the landmark model rejected is therefore not run
   through it again for 3 s when a candidate shows up at about the same place (within half its size) with a palm score
   that is not more than 0.1 higher (`rejectionMemoryMs`, `SAI_HAND_REJECT_MEMORY_MS`, 0 turns it off). The log field
   `skipped` counts them.

### Environment variables (diagnostics)
| variable | effect |
| --- | --- |
| `SAI_HAND_TRACKING_DEBUG` | print timing statistics and device load every 2 s |
| `SAI_HAND_LOG=path.jsonl` | write every hand result as a JSON line (`palm_best`, `lm_best`, `roi` and `palm_crop` show the best palm and landmark scores, the first ROI and the crop of the palm run, for diagnosing missed hands) |
| `SAI_HAND_REPLAY=color.raw` | feed recorded frames (see below) instead of the camera, `SAI_HAND_REPLAY_REALTIME=1` paces them, `SAI_HAND_REPLAY_LOOP=1` restarts at the end |
| `SAI_HAND_NO_DEPTH`, `SAI_IR_PROJECTOR=0..1` | examples: disable depth, IR projector intensity |
| `SAI_RECORD_DIR`, `SAI_RUN_SECONDS`, `SAI_RECORD_NO_COLOR` | example: record `color.raw` + `vio.jsonl`, run time, skip the color frames |
| `SAI_USE_COLOR`, `SAI_USE_STEREO`, `SAI_USE_FEATURE_TRACKER`, `SAI_FAST_VIO`, `SAI_LOW_LATENCY` | example: toggle Spectacular AI components |
| `SAI_HAND_REJECT_MEMORY_MS` | how long (ms) a rejected palm candidate is not run through the landmark model again at the same place, default 3000, 0 = off |
| `SAI_HAND_SEARCH_CROPS=0` | palm search on the whole frame only (the old behavior); by default it cycles through crops |
| `SAI_HAND_SEARCH_INTERVAL_MS`, `SAI_HAND_THROTTLE_START_C` | minimum time between palm detections while no hand is tracked, chip temperature where the work is reduced |
| `SAI_HAND_BENCH`, `SAI_HAND_DUMP_NODES`, `SAI_HAND_NO_PREVIEW`, `SAI_HAND_LM_THREADS` ... | pipeline experiments, see `depthai.cpp` |

### Testing without a camera in front of you
`tools/hand_tracking` has scripts for a record, replay and compare workflow:
 1. `run_recording.ps1 -OutDir rec` runs `main_depthai` with spoken instructions and records the color frames
    (`color.raw`), the VIO poses and camera intrinsics.
 2. `ref_python_tracker.py rec rec/ref.jsonl --lm full` runs the original Python tracker on the same frames.
 3. Replay the recording through the plugin with `SAI_HAND_REPLAY=rec\color.raw SAI_HAND_LOG=rec\native.jsonl`
    (`main_depthai.exe palm.blob landmark.blob`) and compare with `compare_tracks.py rec/ref.jsonl rec/native.jsonl`,
    or look at it with `render_overlay.py`. On a 55 s recording both trackers agree to ~0.85 px mean landmark error.
 4. `main_hand_tracking.exe` runs the unit tests of the algorithms.
 5. Live tests with spoken prompts and analysis: `guided_tests.ps1` (`-Part vio|player|world`: stationary VIO noise,
    Unity player hands test, static hand and ruler distance test), `analyze_player.py`, `analyze_world.py`,
    `thermal_runs.ps1` (chip temperature over time), `run_replay.ps1`. Their paths are set with environment
    variables, see the top of `guided_tests.ps1`.
    `pinch_session.ps1 -OutDir pinch1` is a spoken session of pinches and other poses (13 poses, 2.5 min) and
    `analyze_pinch.py pinch1` compares thumb-index features and thresholds for the pinch gesture.

## Run C++ examples (for debugging/testing)
1. Live example with DepthAI devices. Connect DepthAI device and then run
```
.\Release\main_depthai.exe path\to\palm_detection_sh4.blob path\to\hand_landmark_full_sh4.blob
```
A window shows the camera image with the tracked hands. The terminal prints hand tracking latency and the position
of the device. Without the blob arguments only the VIO runs.

2. If you have recorded datasets, then you can replay them using
```
.\Release\main_replay.exe path\to\recording
```
The position of the device should be printed in your terminal.
