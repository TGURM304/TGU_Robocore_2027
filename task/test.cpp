#include <iostream>
#include <toml.hpp>

int main() {
    auto config = toml::parse_file("../../config/testconfig.toml");

    std::string title = config["title"].value_or("default");

    // int id = config["robot"]["id"].value_or(0);
    // double speed = config["robot"]["speed"].value_or(0.0);
    //
    // std::cout << title << std::endl;
    // std::cout << "id: " << id << std::endl;
    // std::cout << "speed: " << speed << std::endl;

    auto arr1 = config["camera"]["camera_matrix"].as_array();

    double K[3][3];

    for (int i = 0; i < 9; i++) {
        K[i / 3][i % 3] = (*arr1)[i].value_or(0.0);
    }

    // 打印
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << K[i][j] << " ";
        }
        std::cout << std::endl;
    }

    auto arr = config["camera"]["distort_coeffs"].as_array();

    if (!arr || arr->size() != 5) {
        throw std::runtime_error("distort_coeffs must be size 5");
    }

    for (size_t i = 0; i < 5; i++) {
        double val = (*arr)[i].value_or(0.0);
        std::cout << val << std::endl;
    }

    return 0;
}