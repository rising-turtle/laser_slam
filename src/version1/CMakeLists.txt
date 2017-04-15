SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
SET(QT_NETWORK "/usr/include/qt4/QtNetwork")
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_NETWORK} 
	${FLIRT_DIR} ${EIGEN}
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm
	${CMAKE_CURRENT_SOURCE_DIR}/../mapGraph
	${CMAKE_CURRENT_SOURCE_DIR}/../slam
	${CMAKE_CURRENT_SOURCE_DIR}/../zhcsm
	${CMAKE_CURRENT_SOURCE_DIR}/../sick_reader
	${CMAKE_CURRENT_SOURCE_DIR}/../timestamp
	${CMAKE_CURRENT_SOURCE_DIR}/../sensorFusion
	${CMAKE_CURRENT_SOURCE_DIR}/../Main-Ctrl
	${CMAKE_CURRENT_SOURCE_DIR}/../localization
	)
SET(CLIENT_SRCS
	slam_v1.cpp
	dialog_client.cpp
	clientLocal.cpp
	clientFusion.cpp
	clientFrontend.cpp
	clientBackend.cpp
	frontendSICK.cpp
	)
	
SET(SERVER_SRCS
	points.cpp
	dialog_server.cpp
	serverGlobal.cpp
	serverSocket.cpp
	serverBackend.cpp
	)

SET(CLIENT_MOC_HDRS
	slam_v1.h
	dialog_client.h
	clientLocal.h
	clientFusion.h
	clientFrontend.h
	clientBackend.h
	frontendSICK.h
)

SET(SERVER_MOC_HDRS
	points.h
	dialog_server.h
	serverGlobal.h
	serverSocket.h
	serverBackend.h
	)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
QT4_WRAP_CPP(CLIENT_MOC_SRCS ${CLIENT_MOC_HDRS})
ADD_LIBRARY(client_slam_v1 SHARED ${CLIENT_SRCS} ${CLIENT_MOC_SRCS})

QT4_WRAP_CPP(SERVER_MOC_SRCS ${SERVER_MOC_HDRS})
ADD_LIBRARY(server_slam_v1 SHARED ${SERVER_SRCS} ${SERVER_MOC_SRCS})

LINK_DIRECTORIES(${EXTERNAL_LINKS} ${CSM_LIB} ${CMAKE_BINARY_DIR}/lib /usr/lib/i386-linux-gnu)
LINK_DIRECTORIES(${G2O_LIB} ${FLIRT_LIB} ${SSPARSE_DIR} ${MRPT_LIB})

SET(OUR_LIBS slam timestamp localization drawmap sensorFusion zhpsm pfg zhicp zhcsm sick_reader)

SET(CLIENT_LIBS ${OUR_LIBS} ${CSM_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${MRPT_LIBS})

#MESSAGE(STATUS, "QT_LIBRARIES: ${QT_LIBRARIES}")


ADD_EXECUTABLE(client_v1 main_client.cpp)
TARGET_LINK_LIBRARIES(client_v1 client_slam_v1 ${CLIENT_LIBS} ${OPENGL_LIBS} ${QT_LIBS1})

ADD_EXECUTABLE(test_v1 test_v1.cpp)
TARGET_LINK_LIBRARIES(test_v1 client_slam_v1 ${CLIENT_LIBS} ${OPENGL_LIBS} ${QT_LIBS1})

# SET(SERVER_LIBS ${OUR_LIBS} ${CSM_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${SSPARSE_LIBS})
ADD_EXECUTABLE(server_v1 main_server.cpp)
TARGET_LINK_LIBRARIES(server_v1 server_slam_v1 zhpsm ${OPENGL_LIBS} ${QT_LIBS1})


