cmake_minimum_required(VERSION 3.14)
project(chrono_flap_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Chrono was built with C++20; must match to avoid ABI mismatches.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Chrono REQUIRED)

# Collect libraries
list(APPEND LIBS Chrono_core)

if(Chrono_VSG_AVAILABLE)
  find_library(CHRONO_VSG_LIB Chrono_vsg HINTS /usr/local/lib)
  if(CHRONO_VSG_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_vsg at ${CHRONO_VSG_LIB}")
    list(APPEND LIBS ${CHRONO_VSG_LIB})

    # Find VSG include dir from CMAKE_PREFIX_PATH.
    # We intentionally do NOT link vsg::vsg because the imported target adds
    # compile definitions that trigger a GCC 13 <format> template bug.
    # Instead we just need the include path for vsg/all.h.
    set(_VSG_INCLUDE_DIR "")
    foreach(_prefix ${CMAKE_PREFIX_PATH})
      if(EXISTS "${_prefix}/include/vsg/all.h")
        set(_VSG_INCLUDE_DIR "${_prefix}/include")
        break()
      endif()
    endforeach()

    if(_VSG_INCLUDE_DIR)
      message(STATUS "chrono_flap_sim: Found VSG headers at ${_VSG_INCLUDE_DIR}")
      set(VSG_EXTRA_INCLUDES ${_VSG_INCLUDE_DIR})
    else()
      message(WARNING "chrono_flap_sim: Chrono_vsg found but vsg/all.h not found. "
        "Set CMAKE_PREFIX_PATH to the vsg install prefix.")
    endif()

    # Link vsgXchange/vsgImGui libs (but not the imported targets)
    find_library(_VSGXCHANGE_LIB vsgXchange HINTS /usr/local/lib)
    if(_VSGXCHANGE_LIB)
      list(APPEND LIBS ${_VSGXCHANGE_LIB})
    endif()
  else()
    message(STATUS "chrono_flap_sim: Chrono_VSG_AVAILABLE but library not found")
  endif()
elseif(Chrono_IRRLICHT_AVAILABLE)
  find_library(CHRONO_IRRLICHT_LIB Chrono_irrlicht HINTS /usr/local/lib)
  if(CHRONO_IRRLICHT_LIB)
    message(STATUS "chrono_flap_sim: Found Chrono_irrlicht at ${CHRONO_IRRLICHT_LIB}")
    list(APPEND LIBS ${CHRONO_IRRLICHT_LIB})
  endif()
else()
  message(STATUS "chrono_flap_sim: No visualization module - running headless")
endif()

add_executable(chrono_flap_node src/chrono_flap_node.cpp)

target_include_directories(chrono_flap_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CHRONO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${VSG_EXTRA_INCLUDES}
)

target_link_libraries(chrono_flap_node ${LIBS})

ament_target_dependencies(chrono_flap_node
  rclcpp
  std_msgs
)

install(TARGETS chrono_flap_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()