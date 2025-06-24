cmake_minimum_required(VERSION 3.21)
project(spectacularAI_unity)

if(MSVC)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wl,--no-as-needed")
endif()

find_package(depthai REQUIRED)
find_package(spectacularAI_depthaiPlugin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)

set(PLUGIN_SRC
  src/replay.cpp
  src/output.cpp
  src/render.cpp
  src/util.cpp
  src/depthai.cpp
  src/mapping.cpp
)

set(PLUGIN_LIBS
  depthai::core
  spectacularAI::depthaiPlugin)

if(MSVC)
  # Depthai-core needs this and cmake can't find it otherwise
  find_package(usb-1.0 REQUIRED)
  list(APPEND PLUGIN_LIBS usb-1.0)
endif()

add_library(${CMAKE_PROJECT_NAME} SHARED ${PLUGIN_SRC})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/Unity
  /usr/include/eigen3
)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${PLUGIN_LIBS} OpenGL::GL GLEW::GLEW)

# enables searching for dynamic libraries from the relative path
if(NOT MSVC)
  set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE)
endif()

# C++ Replay example
add_executable(main_replay ${PLUGIN_SRC} examples/main_replay.cpp)
target_link_libraries(main_replay PRIVATE ${PLUGIN_LIBS} OpenGL::GL GLEW::GLEW)
target_include_directories(main_replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include "include/spectacularAI/unity" /usr/include/eigen3)

# C++ DepthAI example
add_executable(main_depthai ${PLUGIN_SRC} examples/main_depthai.cpp)
target_link_libraries(main_depthai PRIVATE ${PLUGIN_LIBS} OpenGL::GL GLEW::GLEW)
target_include_directories(main_depthai PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include "include/spectacularAI/unity" /usr/include/eigen3)

if(MSVC)
  add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:${CMAKE_PROJECT_NAME}> $<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>
    COMMAND_EXPAND_LISTS
  )
endif()
