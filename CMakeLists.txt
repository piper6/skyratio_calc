cmake_minimum_required(VERSION 3.15...3.27)
project(skyratio_calc)

# C++17を有効化
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Python開発環境を検索
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

# nanobindの設定
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/ext/nanobind)

# tiny_bvhのインクルードパス
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_compile_options(-Wall -Wextra)

# C++サンプル実行ファイル
add_executable(sample_cpp 
    src/sample_cpp.cpp
    src/scene_raycaster.cpp
    src/sky_ratio_checker.cpp
)

add_executable(repro_false_negative
    src/repro_false_negative.cpp
    src/scene_raycaster.cpp
)

# Pythonモジュールの作成
nanobind_add_module(skyratio_calc 
    src/hello.cpp
    src/scene_raycaster.cpp
    src/sky_ratio_checker.cpp
)

# インストール設定
install(TARGETS skyratio_calc LIBRARY DESTINATION .)
install(TARGETS sample_cpp RUNTIME DESTINATION .)
install(TARGETS repro_false_negative RUNTIME DESTINATION .)

# スタブファイルのインストール
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/skyratio_calc.pyi DESTINATION .)

# Enable AddressSanitizer and MemorySanitizer for Debug builds
# if (CMAKE_BUILD_TYPE STREQUAL "Debug")
#     set(SANITIZER_FLAGS "-fsanitize=address -fsanitize=undefined")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZER_FLAGS} -fno-omit-frame-pointer")
#     set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} ${SANITIZER_FLAGS}")
# endif()
