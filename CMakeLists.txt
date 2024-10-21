cmake_minimum_required (VERSION 3.8)

project (statsdcc CXX)

set(DISABLE_PAL_FLAGS TRUE)

find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED regex)

ament_auto_find_build_dependencies()

include(CheckIncludeFileCXX)
include (CheckCXXCompilerFlag)

set (PROJECT_VERSION_MAJOR 0)
set (PROJECT_VERSION_MINOR 14)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION
    "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
execute_process (
  COMMAND git log -1 --format=%h
  OUTPUT_VARIABLE BUILD
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if (NOT debug)
  set (debug "false")
endif()

configure_file (include/statsdcc/version.h.in
                ${PACKAGE_XML_DIRECTORY}/include/statsdcc/version.h)

add_definitions (-D_GLIBCXX_USE_NANOSLEEP)
add_definitions (-D_GLIBCXX_USE_SCHED_YIELD)

add_compile_options (-Wno-deprecated-declarations)

include_directories (
   include
   ${Boost_INCLUDE_DIRS}
   build/include
   # vendor/gtest-1.7.0/include
)
include_directories(SYSTEM)

# check if atomic.h exists
check_include_file_cxx (atomic HAVE_ATOMIC)
check_include_file_cxx (cstdatomic HAVE_CSTDATOMIC)

if (HAVE_ATOMIC)
  add_definitions(-DHAVE_ATOMIC)
endif()

if (HAVE_CSTDATOMIC)
  add_definitions(-DHAVE_CSTDATOMIC)
endif()

# find libraries
function (lib_not_found library_name)
  message (FATAL_ERROR
    "Unable to find ${library_name}. "
    "Try setting the path to ${library_name} in environment variable CMAKE_LIBRARY_PATH."
  )
endfunction(lib_not_found)

find_library (CRYPTO NAMES libcrypto.a crypto)
if (NOT CRYPTO)
  lib_not_found("libcrypto.a")
endif()

find_library (JSONCPP NAMES libjsoncpp.a jsoncpp)
if (NOT JSONCPP)
  lib_not_found("libjsoncpp.a")
endif()

find_library (TCMALLOC_MINIMAL NAMES libtcmalloc_minimal.a tcmalloc_minimal)
if (NOT TCMALLOC_MINIMAL)
  lib_not_found("libtcmalloc_minimal.a")
endif()

find_library (MICROHTTPD NAMES libmicrohttpd.so microhttpd)
if (NOT MICROHTTPD)
  lib_not_found("libmicrohttpd.so")
endif()

add_subdirectory (lib)
add_subdirectory (src)

# if(CATKIN_ENABLE_TESTING)
#   # enable_testing()

#   add_subdirectory (test)
#   # using version packaged with catkin
#   # add_subdirectory (vendor/gtest-1.7.0)

#   # add_test (ledger_test test/ledger_test)

#   if (RUN_NODE_HASHRING_TESTS)
#     add_test (hashring_test test/hashring_test)
#   endif()

# endif()

ament_auto_package()
