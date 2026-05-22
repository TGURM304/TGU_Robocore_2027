cmake_minimum_required(VERSION 3.16)

include(CheckIncludeFileCXX)

find_package(Threads REQUIRED)

check_include_file_cxx(asio.hpp MID360_DRIVER_HAS_STANDALONE_ASIO)

# 创建目标 io
add_library(io STATIC
        serial/serial.cpp
        aravis/aravis.cpp
        hikrobot/hikrobot.cpp
        mid360_driver/mid360_driver.cpp
#        mindvision/mindvision.cpp
#        usbcamera/usbcamera.cpp
#        camera.cpp
#        cboard.cpp
#        dm_imu/dm_imu.cpp
#        gimbal/gimbal.cpp
#        gimbal/gimbal_nav.cpp
)

set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/hikSDK/lib/amd64)

if(MID360_DRIVER_HAS_STANDALONE_ASIO)
        target_compile_definitions(io
                PUBLIC
                ASIO_STANDALONE
                ASIO_NO_DEPRECATED
        )
else()
        target_compile_definitions(io
                PUBLIC
                MID360_DRIVER_USE_BOOST_ASIO
                BOOST_ASIO_NO_DEPRECATED
        )
endif()

target_include_directories(io
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(io
        PUBLIC
#        tools
        Threads::Threads
        boost_system
        pthread
        usb-1.0
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ${ARAVIS_LIBRARIES}
        ${HIK_LIB_DIR}/libMvCameraControl.so
        ${HIK_LIB_DIR}/libMvUsb3vTL.so
#        ${HIK_LIB_DIR}/libMVRender.so
        ${HIK_LIB_DIR}/libMediaProcess.so
)
