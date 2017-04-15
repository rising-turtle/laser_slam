SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
INCLUDE(${QT_USE_FILE})

# find CSM libs
INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/../drawmap
	${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm
	${CMAKE_CURRENT_SOURCE_DIR}/../zhcsm
	${CMAKE_CURRENT_SOURCE_DIR}/../zhicp
	${CMAKE_CURRENT_SOURCE_DIR}/../sensorFusion
	${EIGEN}
	)
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/lib ${MRPT_LIB} ${CSM_LIB})

SET(LOC_SRC
	mainwin.cpp
	pixmap.cpp
	VPmap.cpp
	localization.cpp
	localv1.cpp
	globaldef.cpp
	particles.cpp
	points.cpp
)
SET(MOC_HDRS 
	mainwin.h
	pixmap.h
	points.h
	localization.h
	localv1.h
	)
QT4_WRAP_CPP(MOC_SRCS ${MOC_HDRS})

# out lib path
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
ADD_LIBRARY(localization SHARED ${LOC_SRC} ${MOC_SRCS})

ADD_EXECUTABLE(loc main.cpp)
TARGET_LINK_LIBRARIES(loc localization sensorFusion drawmap zhpsm zhicp zhcsm ${CSM_LIBS} ${MRPT_LIBS} ${QT_LIBRARIES} )

SET(PATH_SRC
	testDiag.cpp
	testState.cpp
	points.cpp
	State_main.cpp

)
SET(PATH_MOC_HDRS 
	testDiag.h
	points.h
	)
QT4_WRAP_CPP(PATH_MOC_SRCS ${PATH_MOC_HDRS})


ADD_EXECUTABLE(testState ${PATH_SRC} ${PATH_MOC_SRCS})
TARGET_LINK_LIBRARIES(testState zhpsm ${OPENGL_LIBS} ${QT_LIBRARIES} )

ADD_EXECUTABLE(testRandom testRandom.cpp)
ADD_EXECUTABLE(testRandom2 test_2.cpp)
# TARGET_LINK_LIBRARIES(testRandom )

