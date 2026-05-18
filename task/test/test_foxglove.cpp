//
// Created by Interweave on 2026/4/14.
//

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
    while (true) {
        sleep(1);
    }
}
