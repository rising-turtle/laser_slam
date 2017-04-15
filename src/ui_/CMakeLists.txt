SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
INCLUDE(${QT_USE_FILE})

INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR} 
	${CMAKE_SOURCE_DIR}/include 
	${CMAKE_CURRENT_SOURCE_DIR}/../mapGraph 
	${CMAKE_CURRENT_SOURCE_DIR}/../slam 
	${CMAKE_CURRENT_SOURCE_DIR}/../timestamp 
	${CMAKE_CURRENT_SOURCE_DIR}/../sensorFusion
	${QT_QTOPENGL_INCLUDE_DIR}
	)


SET(INTELMAP_SRCS
	main.cpp
	points.cpp
	gen_points.cpp
	map2d.cpp
	line.cpp
	point2d.cpp
	ui_win.cpp
	graphics.cpp
	imagemap.cpp
)

SET(INTELMAP_MOC_HDRS
	points.h
	gen_points.h
	map2d.h
	ui_win.h
	graphics.h
	imagemap.h
)

# SET Libraries
SET(OTHER_LIBS slam sick_reader sensorFusion timestamp pfg zhicp zhpsm zhcsm)
LINK_DIRECTORIES(${G2O_LIB} ${FLIRT_LIB} ${CSM_LIB} ${CMAKE_BINARY_DIR}/lib ${MRPT_LIB})
QT4_WRAP_CPP(INTELMAP_MOC_SRCS ${INTELMAP_MOC_HDRS})

ADD_EXECUTABLE(ui2 ${INTELMAP_SRCS} ${INTELMAP_MOC_SRCS})
TARGET_LINK_LIBRARIES(ui2 ${OTHER_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${CSM_LIBS} ${OPENGL_LIBS} ${QT_LIBRARIES} ${MRPT_LIBS})

