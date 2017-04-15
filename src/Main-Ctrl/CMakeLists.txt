PROJECT(PFG)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(
	${CMAKE_CURRENT_SOURCE_DIR}/../version1
	${CMAKE_CURRENT_SOURCE_DIR}/../sick_reader
	${CMAKE_CURRENT_SOURCE_DIR}/../timestamp
)

set(SRC_BN
 BN/BN.cpp
 BN/BN.h
 BN/BNpos.cpp
 BN/BNpos.h
)

set(SRC_GPS
 GPS/CGPS.cpp
 GPS/CGPS.h
)

set(SRC_LOG
 LogFile/LogFile.cpp
 LogFile/LogFile.h
)

set(SRC_NET
 NetPortal/NetPortal.cpp
 NetPortal/NetPortal.h
 NetPortal/ClientNet.cpp
 NetPortal/ClientNet.h
 NetPortal/LaserCom.cpp
 NetPortal/LaserCom.h
)

set(SRC_SLAM
 SLAM/SLAM.cpp
 SLAM/SLAM.h
)

set(SRC_SUBCTRL
 #SubCtrlCom/SerialPort.cpp
 #SubCtrlCom/SerialPort.h
 #SubCtrlCom/circbuff.h
 #SubCtrlCom/circbuff.c
 SubCtrlCom/SubCtrlCom.h
 SubCtrlCom/SubCtrlCom.cpp
 #SubCtrlCom/IICtrl.h
 SubCtrlCom/SerialCom/JetFire.cpp
 SubCtrlCom/SerialCom/JetFire.h
 SubCtrlCom/SerialCom/SerialCom.h
 SubCtrlCom/SerialCom/SerialCom.cpp
)

set(SRC_TASK
 Task/_2DMap.cpp
 Task/_2DMap.h
 Task/IOA.cpp
 Task/IOA.h
 Task/PathPlanning.cpp
 Task/PathPlanning.h
 Task/Task.cpp
 Task/Task.h
 Task/Trajectory.cpp
 Task/Trajectory.h
 Task/Pace_Lock.cpp
 Task/Pace_Lock.h
 Task/BMPLib.h
 Task/BMPLib.cpp
)

set(SRC_MAIN
 C_C.h
 C_C.cpp
 MainCtrl_Define.h
 ParseXML.h
 ParseXML.cpp
)

set(SRC_MAPSERVICE
 MapService/AmbientGridMap.h
 MapService/AmbientGridMap.cpp
 MapService/LocalMapBuilder.h
 MapService/LocalMapBuilder.cpp
)

set(SRC_DEMO
 ReadSLAMResult/ReadSLAMResult.h
 ReadSLAMResult/ReadSLAMResult.cpp
)
set(LIB_MC_DEP
 ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdParty/libRX86.so
 pthread
)



LINK_DIRECTORIES(${EXTERNAL_LINKS} ${CSM_LIB} ${CMAKE_BINARY_DIR}/lib /usr/lib/i386-linux-gnu ${G2O_LIB} ${FLIRT_LIB} ${SSPARSE_DIR} ${MRPT_LIB})
SET(OUR_LIBS client_slam_v1 slam timestamp localization drawmap sensorFusion zhpsm pfg zhicp zhcsm sick_reader)

SET(QT_LIBS QtOpenGL QtNetwork QtCore QtGui)
SET(MC_LIBS ${OUR_LIBS} ${CSM_LIBS} ${SSPARSE_LIBS} ${G2O_LIBS} ${FLIRT_LIBS} ${MRPT_LIBS})

ADD_EXECUTABLE(Main-Ctrl ${SRC_BN} ${SRC_TASK} ${SRC_SUBCTRL} ${SRC_SLAM} ${SRC_NET} ${SRC_MAIN} ${SRC_LOG} ${SRC_GPS} ${SRC_DEMO} ${SRC_MAPSERVICE})
target_link_libraries (Main-Ctrl ${LIB_MC_DEP} ${MC_LIBS} ${QT_LIBS} ${MRPT_LIBS})

