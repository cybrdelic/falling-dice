cmake_minimum_required(VERSION 3.20)
project(falling_dice_research LANGUAGES CXX)

option(BUILD_FALLING_DICE "Build the 1,000-dice project" ON)
option(BUILD_METAMATERIAL_FEM "Build the nonlinear metamaterial project" ON)

include(CTest)

if(BUILD_FALLING_DICE)
  add_subdirectory(projects/falling-dice/current)
endif()
if(BUILD_METAMATERIAL_FEM)
  add_subdirectory(projects/metamaterial-compression/industry-fem)
endif()
