SET(CMAKE_BUILD_TYPE DEBUG)

SET(SOURCES
  quickcfg.h
  dttypes.h
  BinningGrid.h
  PointCloud.h
  DeltaTData.h
  OccupancyGrid.h
  PoseData.h
  Utils.h
  
  BinningGrid.cpp
  PointCloud.cpp
  DeltaTData.cpp
  OccupancyGrid.cpp
  PoseData.cpp
  Utils.cpp
  
  mb_fbt.h
  mb_rt.cpp
  mb_angle.cpp
  mb_fbt.cpp
)

ADD_DEFINITIONS( -D_SECURE_SCL=0 -D_HAS_ITERATOR_DEBUGGING=0)

ADD_LIBRARY(dttcore ${SOURCES})

TARGET_LINK_LIBRARIES(dttcore
)


