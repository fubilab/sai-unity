#include "../include/spectacularAI/unity/depthai.hpp"

#include <spectacularAI/output.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <array>

int main(int argc, char *argv[]) {
    ConfigurationWrapper config;
    config.lowLatency = true;
    config.useColor = true;

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
    while (counter < 1000) {
        ColorFrameWrapper* colorFrame = sai_depthai_session_get_color_frame(session);
        if (colorFrame != nullptr) {
            int64_t sequence = sai_color_frame_get_sequence_number(colorFrame);
            if (sequence != lastColorSequence) {
                std::cout << "color frame " << sequence << " ("
                    << sai_color_frame_get_width(colorFrame) << "x"
                    << sai_color_frame_get_height(colorFrame) << ")" << std::endl;
                lastColorSequence = sequence;
            }
            sai_color_frame_release(colorFrame);
        }

        HandTrackingOutputWrapper* handOutput =
            sai_depthai_session_get_hand_tracking_output(session);
        if (handOutput != nullptr) {
            int64_t sequence = sai_hand_tracking_output_get_sequence_number(handOutput);
            if (sequence != lastPalmSequence) {
                std::cout << "palm output " << sequence << " ("
                    << sai_hand_tracking_output_get_count(handOutput)
                    << " detections)" << std::endl;
                lastPalmSequence = sequence;
            }
            sai_hand_tracking_output_release(handOutput);
        }

        if (sai_depthai_session_has_output(session)) {
            ++counter;
            VioOutputWrapper* output = sai_depthai_session_get_output(session);
            spectacularAI::Pose pose = sai_vio_output_get_pose(output);
            std::cout << counter << ". position = " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << std::endl;
            sai_vio_output_release(output); // must release memory!
        }
    }

    sai_depthai_session_release(session); // must release memory!
    sai_depthai_pipeline_release(pipeline); // must release memory!

    return 0;
}
