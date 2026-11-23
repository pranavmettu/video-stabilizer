cmake_minimum_required(VERSION 3.20)
project(video_stabilizer LANGUAGES CXX)

# Use the C++17 standard.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Writes build/compile_commands.json so clangd (VS Code) knows our include paths.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find OpenCV (installed by Homebrew). This sets OpenCV_INCLUDE_DIRS and OpenCV_LIBS.
find_package(OpenCV REQUIRED)

# Build one program called "stabilize" from these source files.
# As we add files (feature_tracker.cpp, ...), we list them here.
add_executable(stabilize
    src/main.cpp
    src/feature_tracker.cpp
    src/motion_estimator.cpp
)

target_include_directories(stabilize PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(stabilize PRIVATE ${OpenCV_LIBS})

# Turn on helpful compiler warnings.
target_compile_options(stabilize PRIVATE -Wall -Wextra)
