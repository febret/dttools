FIND_PACKAGE(MPI REQUIRED)
INCLUDE_DIRECTORIES(${MPI_INCLUDE_PATH})
SET(CMAKE_BUILD_TYPE DEBUG)
ADD_EXECUTABLE(mpidtmerge mpidtmerge.cpp)
ADD_DEFINITIONS( -D_SECURE_SCL=0 -D_HAS_ITERATOR_DEBUGGING=0 -DMPICH_IGNORE_CXX_SEEK)
TARGET_LINK_LIBRARIES(mpidtmerge dttcore ${MPI_LIBRARIES})


