cmake_minimum_required(VERSION 3.16)

# 创建目标 io
add_library(io STATIC
        serial/serial.cpp
        aravis/aravis.cpp
        hikrobot/hikrobot.cpp
#        mindvision/mindvision.cpp
#        usbcamera/usbcamera.cpp
#        camera.cpp
#        cboard.cpp
#        dm_imu/dm_imu.cpp
#        gimbal/gimbal.cpp
#        gimbal/gimbal_nav.cpp
)

set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikrobot/hikSDK/lib/amd64)

target_link_libraries(io
        PUBLIC
#        tools
        boost_system
        pthread
        usb-1.0
        ${OpenCV_LIBS}
        ${ARAVIS_LIBRARIES}
        ${HIK_LIB_DIR}/libMvCameraControl.so
        ${HIK_LIB_DIR}/libMvUsb3vTL.so
#        ${HIK_LIB_DIR}/libMVRender.so
        ${HIK_LIB_DIR}/libMediaProcess.so
)