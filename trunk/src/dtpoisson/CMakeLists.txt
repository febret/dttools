SET(CMAKE_BUILD_TYPE DEBUG)
if(NOT WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
endif()
SET(SOURCES
	Factor.cpp
	Geometry.cpp
	MarchingCubes.cpp
	Time.cpp
	ply.cpp
	plyfile.cpp
	dtpoisson.cpp
)

ADD_DEFINITIONS( -D_SECURE_SCL=0 -D_HAS_ITERATOR_DEBUGGING=0)
ADD_EXECUTABLE(dtpoisson ${SOURCES})
TARGET_LINK_LIBRARIES(dtpoisson dttcore)


