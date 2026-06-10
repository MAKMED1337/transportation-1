cmake_minimum_required(VERSION 3.24)
project(transport_routing LANGUAGES CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
include(CTest)

FetchContent_Declare(
  protozero
  GIT_REPOSITORY https://github.com/mapbox/protozero.git
  GIT_TAG v1.7.1
  SOURCE_SUBDIR cmake-no-add-subdirectory
)
FetchContent_Declare(
  libosmium
  GIT_REPOSITORY https://github.com/osmcode/libosmium.git
  GIT_TAG v2.22.0
  SOURCE_SUBDIR cmake-no-add-subdirectory
)
FetchContent_MakeAvailable(protozero libosmium)

find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings INTERFACE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion>
)

add_library(transport_core
  src/algorithms/astar.cpp
  src/algorithms/ch/contraction_hierarchy.cpp
  src/algorithms/ch/node_order.cpp
  src/algorithms/ch/witness_search.cpp
  src/algorithms/ch/work_graph.cpp
  src/algorithms/dijkstra.cpp
  src/graph/graph.cpp
  src/algorithms/routing_algorithm_factory.cpp
)
target_compile_features(transport_core PUBLIC cxx_std_23)
target_include_directories(transport_core PUBLIC src)
target_link_libraries(transport_core PUBLIC Threads::Threads PRIVATE project_warnings)

add_executable(transport_import_osm src/apps/import_main.cpp)
target_include_directories(transport_import_osm SYSTEM PRIVATE ${libosmium_SOURCE_DIR}/include ${protozero_SOURCE_DIR}/include)
target_link_libraries(transport_import_osm PRIVATE transport_core project_warnings ZLIB::ZLIB BZip2::BZip2 EXPAT::EXPAT)

add_executable(transport_query src/apps/query_main.cpp)
target_link_libraries(transport_query PRIVATE transport_core project_warnings)

add_executable(transport_benchmark src/apps/benchmark_main.cpp)
target_link_libraries(transport_benchmark PRIVATE transport_core project_warnings)

if(BUILD_TESTING)
  add_executable(transport_routing_tests tests/routing_algorithms_test.cpp)
  target_link_libraries(transport_routing_tests PRIVATE transport_core project_warnings)
endif()

add_executable(write_benchmark_graph src/apps/write_benchmark_graph.cpp)
target_link_libraries(write_benchmark_graph PRIVATE transport_core project_warnings)

add_executable(ch_measure src/apps/ch_measure_main.cpp)
target_link_libraries(ch_measure PRIVATE transport_core project_warnings)

if(BUILD_TESTING)
  add_test(NAME query_help COMMAND transport_query --help)
  add_test(NAME routing_algorithms COMMAND transport_routing_tests)
endif()
