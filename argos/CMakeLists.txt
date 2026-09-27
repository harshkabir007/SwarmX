cmake_minimum_required(VERSION 3.16)
project(swarmx_argos CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ARGoS3 installed from source: pkg-config if available, otherwise SWARMX_ARGOS_PREFIX (default /usr/local)
set(SWARMX_ARGOS_PREFIX "/usr/local" CACHE PATH "ARGoS3 install prefix")
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(ARGOS QUIET argos3_simulator)
endif()
if(NOT ARGOS_FOUND)
  set(ARGOS_INCLUDE_DIRS ${SWARMX_ARGOS_PREFIX}/include)
  set(ARGOS_LIBRARY_DIRS ${SWARMX_ARGOS_PREFIX}/lib/argos3)
endif()
# ARGoS headers include lua.h (Lua 5.3 on Ubuntu 24.04)
find_path(LUA_INCLUDE_DIR lua.h PATHS /usr/include/lua5.3 /usr/include/lua5.4 /usr/include/lua /usr/local/include)
include_directories(${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR})
link_directories(${ARGOS_LIBRARY_DIRS})
add_compile_definitions(ARGOS_USE_DOUBLE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

foreach(lib argos3core_simulator argos3plugin_simulator_footbot argos3plugin_simulator_genericrobot
            argos3plugin_simulator_entities)
  find_library(LIB_${lib} ${lib} PATHS ${ARGOS_LIBRARY_DIRS} ${SWARMX_ARGOS_PREFIX}/lib/argos3 REQUIRED)
endforeach()

add_library(swarmx_footbot MODULE controllers/swarmx_footbot/swarmx_footbot.cpp)
target_link_libraries(swarmx_footbot ${LIB_argos3core_simulator} ${LIB_argos3plugin_simulator_footbot}
                      ${LIB_argos3plugin_simulator_genericrobot})

add_library(swarmx_loop_functions MODULE loop_functions/swarmx_loop_functions.cpp)
target_link_libraries(swarmx_loop_functions ${LIB_argos3core_simulator} ${LIB_argos3plugin_simulator_footbot}
                      ${LIB_argos3plugin_simulator_entities})
