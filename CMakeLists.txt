cmake_minimum_required(VERSION 3.10)

project(miao)

# 独立构建 miao 图优化库及其示例
# 依赖：Eigen3、Sophus（deps/Sophus，header-only）、glog、gflags（仅示例）、TBB

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Eigen3（找不到时回退到系统默认路径）
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
endif()

# 头文件搜索路径：
#   ${CMAKE_SOURCE_DIR}       -> core/...  utils/sampler.h 等（库自身）
#   ${CMAKE_SOURCE_DIR}/deps  -> common/eigen_types.h、utils/timer.h、Sophus/...（复制的依赖）
include_directories(
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/deps
        ${EIGEN3_INCLUDE_DIR}
)

# 兼容示例代码中的 miao:: 前缀（见 deps/compat.h）
add_compile_options(-include ${CMAKE_SOURCE_DIR}/deps/compat.h)

add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(examples)
