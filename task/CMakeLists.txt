# test
add_executable(read_toml test/read_toml.cpp)
add_executable(test_serial test/test_serial.cpp)
add_executable(test_logger test/test_logger.cpp)
add_executable(test_foxglove test/test_foxglove.cpp)
add_executable(test_camera test/test_camera.cpp)
add_executable(test_mid360 test/test_mid360.cpp)

target_link_libraries(test_serial PRIVATE io tools)
target_link_libraries(test_logger PRIVATE tools)
target_link_libraries(test_foxglove PRIVATE tools ${OpenCV_LIBS})
target_link_libraries(test_camera PRIVATE tools ${OpenCV_LIBS} io)
target_link_libraries(test_mid360 PRIVATE tools io)

# task
add_executable(sentry sentry.cpp)

target_link_libraries(sentry PRIVATE tools io ${OpenCV_LIBS} )
