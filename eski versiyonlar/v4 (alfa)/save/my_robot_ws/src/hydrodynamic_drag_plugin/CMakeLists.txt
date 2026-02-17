cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(hydrodynamic_drag_plugin)

# Gazebo kütüphanelerini bul (alternatif sürümler dene)
find_package(gz-sim8 QUIET)
if(NOT gz-sim8_FOUND)
  find_package(gz-sim7 QUIET)
  if(NOT gz-sim7_FOUND)
    find_package(ignition-gazebo6 REQUIRED)
    set(GZ_SIM_VER ignition-gazebo6)
  else()
    set(GZ_SIM_VER gz-sim7)
  endif()
else()
  set(GZ_SIM_VER gz-sim8)
endif()

find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 QUIET)
if(NOT gz-math7_FOUND)
  find_package(gz-math6 REQUIRED)
  set(GZ_MATH_VER gz-math6)
else()
  set(GZ_MATH_VER gz-math7)
endif()

message(STATUS "Using ${GZ_SIM_VER}")
message(STATUS "Using ${GZ_MATH_VER}")

# Plugin'i derle
add_library(HydrodynamicDrag SHARED HydrodynamicDrag.cc)
set_target_properties(HydrodynamicDrag PROPERTIES PREFIX "lib")

target_link_libraries(HydrodynamicDrag
  ${GZ_SIM_VER}::${GZ_SIM_VER}
  gz-plugin2::gz-plugin2
  ${GZ_MATH_VER}::${GZ_MATH_VER}
)

# Kurulum
install(TARGETS HydrodynamicDrag
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)