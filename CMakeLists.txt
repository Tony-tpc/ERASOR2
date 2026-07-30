cmake_minimum_required(VERSION 3.16)
project(erasor2 VERSION 1.1.0 LANGUAGES C CXX)

# Pure-C++ build. Replaces the v1.0 catkin layer; nothing here depends on
# a ROS environment. The only dependency that used to ship as a catkin
# package was grid_map_core/grid_map_cv, which we replaced with a
# self-contained erasor2::GridMap (include/erasor2/grid_map.hpp).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)

# Debian/Ubuntu PCL is linked against VTK. Some VTK builds export VTK::mpi
# with MPI::MPI_C in their link interface. Because this project originally
# enabled only CXX, FindMPI could not create MPI::MPI_C and CMake generation
# failed. Enable C above and discover the C MPI target before loading PCL/VTK.
find_package(MPI REQUIRED COMPONENTS C)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

# Rerun is optional. It is disabled by default so headless/offline builds do
# not download rerun_cpp_sdk (and its Apache Arrow dependency) during CMake
# configuration. Enable explicitly with -DERASOR2_ENABLE_RERUN=ON.
option(ERASOR2_ENABLE_RERUN "Build Rerun visualization support" OFF)

if(ERASOR2_ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/download/0.32.1/rerun_cpp_sdk.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(rerun_sdk)
    set(ERASOR2_RERUN_SOURCE src/erasor2/RerunLogger.cpp)
else()
    message(STATUS "ERASOR2: Rerun visualization disabled (offline/headless build)")
    set(ERASOR2_RERUN_SOURCE src/erasor2/RerunLoggerStub.cpp)
endif()

# --- Shared static lib used by all erasor2 binaries -------------------------
add_library(erasor2_core STATIC
    src/erasor2/Config.cpp
    ${ERASOR2_RERUN_SOURCE}
    src/erasor2/grid_map.cpp
)
target_include_directories(erasor2_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(erasor2_core SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(erasor2_core PUBLIC
    yaml-cpp
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)
if(ERASOR2_ENABLE_RERUN)
    target_link_libraries(erasor2_core PUBLIC rerun_sdk)
    target_compile_definitions(erasor2_core PUBLIC ERASOR2_ENABLE_RERUN=1)
else()
    target_compile_definitions(erasor2_core PUBLIC ERASOR2_ENABLE_RERUN=0)
endif()

set(erasor2_src
    src/erasor2/erasor_utils.cpp
    src/dataloader/dataloader.cpp
)

add_executable(mapgen src/mapgen/main.cpp ${erasor2_src})
target_link_libraries(mapgen erasor2_core stdc++fs)

add_executable(run_erasor2
    src/erasor2/main.cpp
    src/erasor2/erasor_utils.cpp
    src/erasor2/erasor2.cpp
    src/dataloader/dataloader.cpp
)
target_link_libraries(run_erasor2 erasor2_core OpenMP::OpenMP_CXX stdc++fs)
target_compile_options(run_erasor2 PRIVATE ${OpenMP_CXX_FLAGS})

add_executable(accum_4dmos
    src/accum_4dmos.cpp
    src/erasor2/erasor_utils.cpp
    src/dataloader/dataloader.cpp
)
target_link_libraries(accum_4dmos erasor2_core stdc++fs)

# --- ERASOR (v1) -----------------------------------------------------------
# Port of github.com/LimHyungTae/ERASOR's algorithm into this repo (without
# ROS). Reuses the same dataloader / rerun adapter / utils as v2 so a single
# build produces both binaries.
add_executable(run_erasor
    src/erasor1/main.cpp
    src/erasor1/erasor.cpp
    src/erasor2/erasor_utils.cpp
    src/dataloader/dataloader.cpp
)
target_link_libraries(run_erasor erasor2_core stdc++fs)

add_executable(compare_map src/compare_map.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(compare_map erasor2_core)

add_executable(fill_removert_labels src/fill_removert_labels.cpp src/erasor2/erasor_utils.cpp)
target_link_libraries(fill_removert_labels erasor2_core stdc++fs)

# --- HeLiPR data preprocessing tools ---------------------------------------
set(dataprocessor_src
    src/dataprocessor/visualizer.cpp
    src/dataprocessor/PointCloudProcessor.cpp
    src/dataprocessor/utility.cpp
    src/dataprocessor/BsplineSE3.cpp
)

add_executable(helipr_to_kitti src/dataprocessor/pointSaver.cpp ${dataprocessor_src})
target_link_libraries(helipr_to_kitti erasor2_core OpenMP::OpenMP_CXX)

add_executable(merge_heliclouds src/dataprocessor/pointMerger.cpp ${dataprocessor_src})
target_link_libraries(merge_heliclouds erasor2_core OpenMP::OpenMP_CXX)
