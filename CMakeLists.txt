project(MRPE)

cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# CGAL and its components
find_package(CGAL QUIET COMPONENTS)

if (NOT CGAL_FOUND)

    message(STATUS "This project requires the CGAL library, and will not be compiled.")
    return()

endif ()

# include helper file
include(${CGAL_USE_FILE})


# Boost and its components
find_package(Boost REQUIRED)

if (NOT Boost_FOUND)

    message(STATUS "This project requires the Boost library, and will not be compiled.")

    return()

endif ()

# include for local directory

# include for local package


# Creating entries for target: executable
# ############################

set(SOURCE_FILES CGAL_defines.h main.cpp PathFinder.cpp PathFinder.h ConfigurationPoint.h ConfigurationPoint.cpp WorkingSpace.cpp WorkingSpace.h)

add_executable(MRPE ${SOURCE_FILES})
add_to_cached_list(CGAL_EXECUTABLE_TARGETS MRPE)

# Link the executable to CGAL and third-party libraries
target_link_libraries(MRPE ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})