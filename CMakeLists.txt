cmake_minimum_required (VERSION 3.8)
project (RPIMAIN)

add_subdirectory(src)

include_directories(./src/MPU6500)

# target_link_libraries(RockPiMPU ${RPIM})
add_executable(RockPiMPU "main.cpp")

