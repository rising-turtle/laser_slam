SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QT3SUPPORT TRUE)
SET(QT_NETWORK "/usr/include/qt4/QtNetwork")
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_NETWORK} 
	${FLIRT_DIR} ${EIGEN}
	${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm
	${CMAKE_CURRENT_SOURCE_DIR}/../mapGraph
	${CMAKE_CURRENT_SOURCE_DIR}/../slam
	${CMAKE_CURRENT_SOURCE_DIR}/../zhcsm
	${CMAKE_CURRENT_SOURCE_DIR}/../sick_reader
	${CMAKE_CURRENT_SOURCE_DIR}/../timestamp
	${CMAKE_CURRENT_SOURCE_DIR}/../sensorFusion
	)
SET(CLIENT_SRCS
	dialog_client.cpp
	clientLocal.cpp
	clientFusion.cpp
	clientFrontend.cpp
	frontendSICK.cpp
	frontendOdo.cpp
	)
	
SET(SERVER_SRCS
	points.cpp
	dialog_server.cpp
	serverGlobal.cpp
	serverSocket.cpp
	serverBackend.cpp
	)

SET(CLIENT_MOC_HDRS
	dialog_client.h
	clientLocal.h
	clientFusion.h
	clientFrontend.h
	frontendSICK.h
	frontendOdo.h
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
ADD_LIBRARY(client_slam1 SHARED ${CLIENT_SRCS} ${CLIENT_MOC_SRCS})

QT4_WRAP_CPP(SERVER_MOC_SRCS ${SERVER_MOC_HDRS})
ADD_LIBRARY(server_slam1 SHARED ${SERVER_SRCS} ${SERVER_MOC_SRCS})

LINK_DIRECTORIES(${EXTERNAL_LINKS} ${CSM_LIB} ${CMAKE_BINARY_DIR}/lib /usr/lib/i386-linux-gnu ${MRPT_LIB})

SET(CLIENT_LIBS zhpsm zhcsm ${CSM_LIBS} sick_reader timestamp sensorFusion)
ADD_EXECUTABLE(client1 main_client.cpp)
TARGET_LINK_LIBRARIES(client1 client_slam ${CLIENT_LIBS} ${OPENGL_LIBS} QtNetwork QtCore QtGui)

LINK_DIRECTORIES(${G2O_LIB} ${FLIRT_LIB} ${SSPARSE_DIR})
SET(OUR_LIBS slam timestamp sensorFusion zhpsm pfg zhicp zhcsm sick_reader)
SET(SERVER_LIBS ${OUR_LIBS} ${CSM_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${MRPT_LIBS})
ADD_EXECUTABLE(server1 main_server.cpp)
TARGET_LINK_LIBRARIES(server1 server_slam1 ${SERVER_LIBS} ${OPENGL_LIBS} QtNetwork QtCore QtGui QtOpenGL)

ADD_EXECUTABLE(oneThread main.cpp)
TARGET_LINK_LIBRARIES(oneThread server_slam1 client_slam1 ${SERVER_LIBS} ${CLIENT_LIBS} ${OPENGL_LIBS} QtNetwork QtCore QtGui QtOpenGL)

