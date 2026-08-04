cmake_minimum_required(VERSION 3.20)
project(thousand_dice_practical_tabletop LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_executable(thousand_dice
    src/main.cpp
    src/sim.cpp
    src/render.cpp)
target_include_directories(thousand_dice PRIVATE include)
if(MSVC)
    target_compile_options(thousand_dice PRIVATE /O2 /W4 /permissive-)
else()
    target_compile_options(thousand_dice PRIVATE -O3 -Wall -Wextra -Wpedantic)
    target_link_libraries(thousand_dice PRIVATE pthread)
endif()
