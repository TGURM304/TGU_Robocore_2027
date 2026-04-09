cmake_minimum_required(VERSION 3.16)
project(TGU_Robocore_2027)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

# ===== 子模块 =====
#add_subdirectory(io)
#add_subdirectory(app)
add_subdirectory(task)
add_subdirectory(tools/tomlpp)