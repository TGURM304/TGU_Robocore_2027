// main.cpp

#include "io/hikrobot/hikrobot.hpp"
#include "tools/foxglove_comm.hpp"

#include <iostream>

int main() {
    tools::FoxGloveComm comm("0.0.0.0", 8765);
    if (!comm.is_ok()) {
        std::cerr << "foxglove init failed\n";
        return -1;
    }
    std::cout << "foxglove ready at " << comm.get_host() << ":" << comm.get_port() << "\n";

    comm.create_image_channel("/image");

    io::HikRobot camera("../config/test.toml");

    if (!camera.init()) {
        return -1;
    }

    if (!camera.start()) {
        return -1;
    }

    while (camera.is_running()) {
        cv::Mat image;

        uint64_t timestamp_ns = 0;

        if (!camera.grab(image, timestamp_ns)) {
            continue;
        }

        cv::resize(image, image, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

        comm.publish_image("/image", image, timestamp_ns, "camera_frame");
    }

    return 0;
}
