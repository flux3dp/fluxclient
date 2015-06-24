cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Possion_C)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


message("PoissonRecon!")
set(PR_TARGET PoissonRecon)
add_executable (${PR_TARGET} CmdLineParser.cpp Factor.cpp Geometry.cpp MarchingCubes.cpp PlyFile.cpp PoissonRecon.cpp )
target_link_libraries (${PR_TARGET} ${PCL_LIBRARIES})

message("SurfaceTrimmer!")
set(ST_TARGET SurfaceTrimmer)
add_executable (${ST_TARGET} CmdLineParser.cpp Factor.cpp Geometry.cpp MarchingCubes.cpp PlyFile.cpp SurfaceTrimmer.cpp)
target_link_libraries (${ST_TARGET} ${PCL_LIBRARIES})
