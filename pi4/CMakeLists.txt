cmake_minimum_required(VERSION 3.10)
project(qr_scanner_pi4 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(qr_scanner_pi4 qr_scanner_pi4.cpp)
target_include_directories(qr_scanner_pi4 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(qr_scanner_pi4 PRIVATE ${OpenCV_LIBS} Threads::Threads)

# Pi 4 = Cortex-A72, 64-bit (Raspberry Pi OS 64-bit / Ubuntu). NEON is
# implied by armv8-a on aarch64, no separate -mfpu flag needed there.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_compile_options(qr_scanner_pi4 PRIVATE -O3 -mcpu=cortex-a72 -DNDEBUG)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  # 32-bit Raspberry Pi OS
  target_compile_options(qr_scanner_pi4 PRIVATE -O3 -mcpu=cortex-a72 -mfpu=neon-fp-armv8 -DNDEBUG)
else()
  target_compile_options(qr_scanner_pi4 PRIVATE -O3 -DNDEBUG)
endif()
