cmake_minimum_required(VERSION 3.10)

project(LiDAR_Monitoring)

set(CMAKE_CXX_STANDARD 11)

# 添加PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

# 设置头文件路径
include_directories(
    sdk/include
    sdk/src
    include
)

# 设置sdk源文件路径
file(GLOB SDK_SRC
    "sdk/src/*.cpp"
    "sdk/src/arch/linux/*.cpp"
    "sdk/src/hal/*.cpp"
)
# 生成静态库
add_library(rplidar_sdk STATIC ${SDK_SRC})

# 设置程序源文件路径
file(GLOB SRC
    "src/*.cpp"
)
# 生成可执行文件
add_executable(LiDAR_monitoring ${SRC})

# 链接库
target_link_libraries(LiDAR_monitoring 
    rplidar_sdk
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pthread
)