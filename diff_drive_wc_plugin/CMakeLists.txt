cmake_minimum_required(VERSION 3.8)
project(diff_drive_wc_plugin)

# Gazebo (Harmonic) dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)          # Gazebo Sim 8 (Harmonic)
find_package(gz-math8 REQUIRED)         # Math types
find_package(gz-transport14 REQUIRED)   # Transport & msgs utilities
find_package(gz-msgs10 REQUIRED)        # Protobuf message headers

# ---- SDFormat: accept multiple majors (distro-dependent) ----
set(SDF_CANDIDATES sdformat15 sdformat14 sdformat13)
set(SDF_FOUND FALSE)
foreach(pkg IN LISTS SDF_CANDIDATES)
  find_package(${pkg} QUIET)
  if(${pkg}_FOUND)
    set(SDF_PKG ${pkg})
    set(SDF_FOUND TRUE)
    break()
  endif()
endforeach()
if(NOT SDF_FOUND)
  message(FATAL_ERROR
    "Could not find SDFormat (sdformat13+). "
    "Install libsdformat14-dev or libsdformat13-dev, or set CMAKE_PREFIX_PATH.")
endif()

add_library(${PROJECT_NAME} SHARED
  src/DiffDrive_wc.cc
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME}
  PUBLIC
    gz-sim8::gz-sim8
    gz-math8::gz-math8
    gz-transport14::core
    gz-msgs10::gz-msgs10
    ${SDF_PKG}::${SDF_PKG}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()

