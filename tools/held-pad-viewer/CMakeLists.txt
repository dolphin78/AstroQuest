cmake_minimum_required(VERSION 3.20)
project(held_pad_viewer CXX C)

set(CMAKE_CXX_STANDARD 20)

set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
set(BUILD_CONFORMANCE_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SDK_TESTS OFF CACHE BOOL "" FORCE)
set(DYNAMIC_LOADER OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../shadps4-arm64-main/externals/openxr-sdk
                 openxr-sdk EXCLUDE_FROM_ALL)

# SDL3 reads the gamepad's motion sensors, as in the emulator: built in, only what that needs.
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST_LIBRARY OFF CACHE BOOL "" FORCE)
set(SDL_AUDIO OFF CACHE BOOL "" FORCE)
set(SDL_CAMERA OFF CACHE BOOL "" FORCE)
set(SDL_GPU OFF CACHE BOOL "" FORCE)
set(SDL_RENDER OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../shadps4-arm64-main/externals/sdl3
                 sdl3 EXCLUDE_FROM_ALL)

add_executable(held_pad_viewer main.cpp)
target_compile_definitions(held_pad_viewer PRIVATE _CRT_SECURE_NO_WARNINGS)
target_include_directories(held_pad_viewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
                                                   ${CMAKE_CURRENT_SOURCE_DIR}/../../shadps4-arm64-main/src/core/vr)
target_link_libraries(held_pad_viewer PRIVATE OpenXR::openxr_loader SDL3::SDL3-static d3d11 dxgi
                                              d3dcompiler)
