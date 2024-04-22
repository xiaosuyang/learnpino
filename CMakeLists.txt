find_package(Eigen3)

project(OverviewSimple)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")


message("sssssssssssssssssssssssssssssssssssss")
message("Eigen3 include directory: ${EIGEN3_INCLUDE_DIRS}")
message("xiugai le yi ci")

include_directories( ${EIGEN3_INCLUDE_DIRS})

OPTION(IF_BUILDREDUCE OFF)

add_subdirectory(robotwrapper)

find_package(pinocchio REQUIRED)
add_executable(overview-simple overview-simple.cpp)
target_link_libraries(overview-simple  PRIVATE pinocchio)
target_link_libraries(overview-simple PRIVATE robotwrapper)

if(${IF_BUILDREDUCE})
    add_executable(buildreducedmodel build-reduced-model.cpp)
target_link_libraries(buildreducedmodel  PRIVATE pinocchio::pinocchio)
endif()


