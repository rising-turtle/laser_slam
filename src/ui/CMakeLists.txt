SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${INTELMAP_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm)

# FlirterNode is needed in runPFGLocal
SET(G2O_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/g2o)
SET(EIGEN ${G2O_DIR}/EXTERNAL/eigen3)
SET(CSPARSE ${G2O_DIR}/EXTERNAL/csparse)
SET(FLIRT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/flirtlib/src)
SET(CHOLMOD_DIR /usr/include/suitesparse)
INCLUDE_DIRECTORIES(${G2O_DIR} ${CSPARSE} ${EIGEN} ${CHOLMOD_DIR} ${FLIRT_DIR} ${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../mapGraph)

MESSAGE("FLIRT DIR: ${FLIRT_DIR}")
MESSAGE("G2O DIR: ${G2O_DIR}")

SET(LINKS glut GLU GL)

SET(INTELMAP_SRCS
	main.cpp
	runPFGLocal.cpp
	runPFGGlobal.cpp
	points.cpp
	gen_points.cpp
	map2d.cpp
	line.cpp
	point2d.cpp
	main_window.cpp
	graphics.cpp
	imagemap.cpp
)

SET(INTELMAP_MOC_HDRS
	runPFGLocal.h
	runPFGGlobal.h
	points.h
	gen_points.h
	map2d.h
	main_window.h
	graphics.h
	imagemap.h
)

# SET(LIBRARY_OUTPUT_DIR ${CMAKE_BINARY_DIR}/lib)
# ADD_LIBRARY(window SHARED ${INTELMAP_SRCS} ${INTELMAP_MOC_SRCS})


# SET Libraries
SET(G2O_LIBS g2o_core g2o_math_groups g2o_parser g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp  g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d)
SET(FLIRT_LIBS  utils feature geometry sensors sensorstream)
SET(OTHER_LIBS cholmod )
SET(ALL_LIBS ${OTHER_LIBS}  ${G2O_LIBS} ${FLIRT_LIBS} )

# SET Lib Directory
SET(FLIRT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/flirtlib/lib)
SET(G2O_LIB ${G2O_DIR}/lib)
SET(EXTERNAL_LINKS ${G2O_LIB} ${FLIRT_LIB})

LINK_DIRECTORIES(${EXTERNAL_LINKS} ${CMAKE_BINARY_DIR}/lib)
QT4_WRAP_CPP(INTELMAP_MOC_SRCS ${INTELMAP_MOC_HDRS})
# ADD_EXECUTABLE(ui ${INTELMAP_SRCS} ${INTELMAP_MOC_SRCS})
# TARGET_LINK_LIBRARIES(ui  zhpsm sick_reader timestamp pfg ${LINKS} ${QT_LIBRARIES} ${ALL_LIBS} )
