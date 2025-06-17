cmake_minimum_required(VERSION 3.10)
project(tf_lcm)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ABI compatibility between Linux and Mac is now handled directly in the message files
# by properly casting function pointers

# Use pkg-config to find LCM instead of CMake's find_package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Print LCM library information for debugging
message(STATUS "LCM include dirs: ${LCM_INCLUDE_DIRS}")
message(STATUS "LCM library dirs: ${LCM_LIBRARY_DIRS}")
message(STATUS "LCM libraries: ${LCM_LIBRARIES}")

# Set include and library dirs
include_directories(${LCM_INCLUDE_DIRS})
link_directories(${LCM_LIBRARY_DIRS})

# Ensure we find liblcm.so directly instead of using IMPORTED targets
find_library(LCM_LIB lcm REQUIRED)
find_library(LCM_CORETYPES_LIB lcm-coretypes)

message(STATUS "Found LCM: ${LCM_LIB}")
message(STATUS "Found LCM coretypes: ${LCM_CORETYPES_LIB}")

# Create targets for LCM libraries
add_library(lcm INTERFACE)
target_include_directories(lcm INTERFACE ${LCM_INCLUDE_DIRS})
target_link_libraries(lcm INTERFACE ${LCM_LIB})

add_library(lcm-coretypes INTERFACE)
target_include_directories(lcm-coretypes INTERFACE ${LCM_INCLUDE_DIRS})
target_link_libraries(lcm-coretypes INTERFACE ${LCM_LIB})

# Add pybind11
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# Include directories - Fix the include paths
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_lcm_msgs  # Directory containing message headers
)

# Add library
add_library(tf_lcm
  src/transform_storage.cpp
  src/buffer.cpp
  src/broadcaster.cpp
  src/listener.cpp
)

# Link against LCM
target_link_libraries(tf_lcm lcm lcm-coretypes)

# Add Python bindings
pybind11_add_module(_tf_lcm_py python/tf_lcm_pybind.cpp)
target_link_libraries(_tf_lcm_py PRIVATE tf_lcm)

# Install targets
install(TARGETS tf_lcm _tf_lcm_py
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/python3/site-packages/tf_lcm_py
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Add tests
add_subdirectory(test)
