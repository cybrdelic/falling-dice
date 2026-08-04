cmake_minimum_required(VERSION 3.20)
project(falling_dice_research LANGUAGES CXX)

include(CTest)
option(BUILD_FALLING_DICE "Build the maintained 1,000-die project" ON)
option(BUILD_METAMATERIAL_COMPRESSION "Build the nonlinear metamaterial project" ON)

if(BUILD_FALLING_DICE)
  add_subdirectory(projects/falling-dice/current projects/falling-dice)
endif()

if(BUILD_METAMATERIAL_COMPRESSION)
  add_subdirectory(projects/metamaterial-compression/industry-grade projects/metamaterial-compression)
endif()
