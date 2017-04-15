SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES()

#####################################
# compile libslam.so
####################################


INCLUDE_DIRECTORIES(
	${G2O_DIR} ${CSPARSE} ${EIGEN} ${SSPARSE_DIR} ${FLIRT_DIR} 
	${INTELMAP_SOURCE_DIR} 
	${QT_QTOPENGL_INCLUDE_DIR} 
	${CMAKE_SOURCE_DIR}/include 
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm
	${CMAKE_CURRENT_SOURCE_DIR}/../zhcsm
	${CMAKE_CURRENT_SOURCE_DIR}/../zhicp
	${CMAKE_CURRENT_SOURCE_DIR}/../mapGraph 
	${CMAKE_CURRENT_SOURCE_DIR}/../timestamp 
	${CMAKE_CURRENT_SOURCE_DIR}/../sensorFusion
	)

SET(SLAM_SRCS
	threadLocal1.cpp
	threadLocal2.cpp
	threadMapNode.cpp
	threadGlobal1.cpp
	threadG2OTcp.cpp
	threadFusion.cpp
	threadSICK.cpp
	threadOdo.cpp
	slam.cpp
)

SET(SLAM_MOC_HDRS
	threadLocal1.h
	threadLocal2.h
	threadMapNode.h
	threadGlobal1.h
	threadG2OTcp.h
	threadSICK.h
	threadFusion.h
	threadOdo.h
	slam.h
)

SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
QT4_WRAP_CPP(SLAM_MOC_SRCS ${SLAM_MOC_HDRS})
ADD_LIBRARY(slam SHARED ${SLAM_SRCS} ${SLAM_MOC_SRCS})

#####################################
# compile test.exe 
#####################################


SET(OTHER_LIBS timestamp sensorFusion zhpsm pfg zhicp zhcsm sick_reader)
LINK_DIRECTORIES(${G2O_LIB} ${FLIRT_LIB} ${CSM_LIB} ${CMAKE_BINARY_DIR}/lib ${MRPT_LIB})
ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test slam  ${OTHER_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${CSM_LIBS} ${QT_LIBRARIES} ${MRPT_LIBS})

