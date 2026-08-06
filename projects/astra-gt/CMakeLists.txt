cmake_minimum_required(VERSION 3.20)
project(astra_gt_vehicle_program VERSION 0.8.0 LANGUAGES CXX)

option(ASTRA_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCASCADE REQUIRED)

add_executable(astra_gt src/main.cpp)
target_include_directories(astra_gt SYSTEM PRIVATE ${OpenCASCADE_INCLUDE_DIR})

# Link only the modeling and STEP modules used by this executable. Ubuntu's
# OpenCASCADE package publishes ${OpenCASCADE_LIBRARIES} with DRAW/test modules
# that are not installed by the runtime development packages; using that broad
# list makes a valid headless CAD build fail at link time.
target_link_libraries(astra_gt PRIVATE
  TKernel TKMath TKG2d TKG3d TKGeomBase
  TKBRep TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool TKShHealing TKFillet TKOffset
  TKMesh TKXSBase TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP
)

if(MSVC)
  target_compile_options(astra_gt PRIVATE /O2 /W4 /permissive- /EHsc)
  if(ASTRA_WARNINGS_AS_ERRORS)
    target_compile_options(astra_gt PRIVATE /WX)
  endif()
else()
  target_compile_options(astra_gt PRIVATE -O3 -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
  if(ASTRA_WARNINGS_AS_ERRORS)
    target_compile_options(astra_gt PRIVATE -Werror)
  endif()
endif()

include(CTest)
if(BUILD_TESTING)
  add_test(NAME astra_gt_generate COMMAND astra_gt ${CMAKE_CURRENT_BINARY_DIR}/smoke/astra_gt)
  set_tests_properties(astra_gt_generate PROPERTIES TIMEOUT 900)
endif()
