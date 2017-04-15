PROJECT(PFG)

# QT
SET(QT_LIBS1 QtOpenGL QtNetwork QtGui)

# CSM 
SET(CSM_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/csm-master/deploy/include)
SET(CSM_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/csm-master/deploy/lib)
SET(CSM_LIBS csm csm-static egsl options)

# G2O
SET(G2O_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/g2o)
SET(G2O_LIB ${G2O_DIR}/lib)
SET(G2O_LIBS g2o_core g2o_math_groups g2o_parser g2o_incremental g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp  g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d g2o_interactive)
### ADD g2o_incremental g2o_interactive

SET(EIGEN ${G2O_DIR}/EXTERNAL/eigen3)
SET(CSPARSE ${G2O_DIR}/EXTERNAL/csparse)

# FLIRT
SET(FLIRT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/flirtlib/src)
SET(FLIRT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/flirtlib/lib)
SET(FLIRT_LIBS  utils feature geometry sensors sensorstream)

# OPENGL 
SET(OPENGL_LIBS glut GLU GL)

# SuiteSparse
SET(SSPARSE_DIR /usr/include/suitesparse)
SET(SSPARSE_LIBS cholmod)

# MRPT
SET(MRPT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/mrpt-0.9.6/libs)
SET(MRPT_CONFIG ${MRPT_DIR}/../build/include/mrpt-config/unix)
SET(MRPT_LIB ${MRPT_DIR}/../build/lib)

SET(MRPT_BASE ${MRPT_DIR}/base/include)
SET(MRPT_BAYES ${MRPT_DIR}/bayes/include)
SET(MRPT_GRAPHS ${MRPT_DIR}/graphs/include)
SET(MRPT_HMT ${MRPT_DIR}/hmtslam/include)
SET(MRPT_REACTIVE ${MRPT_DIR}/reactivenav/include)
SET(MRPT_TOPO ${MRPT_DIR}/topography/include)
SET(MRPT_GSLAM ${MRPT_DIR}/graphslam/include)
SET(MRPT_HWD ${MRPT_DIR}/hwdrivers/include)
SET(MRPT_GUI ${MRPT_DIR}/gui/include)
SET(MRPT_MAPS ${MRPT_DIR}/maps/include)
SET(MRPT_OBS ${MRPT_DIR}/obs/include)
SET(MRPT_SC ${MRPT_DIR}/scanmatching/include)
SET(MRPT_SLAM ${MRPT_DIR}/slam/include)
SET(MRPT_VISION ${MRPT_DIR}/vision/include)
SET(MRPT_DETECTOR ${MRPT_DIR}/detectors/include)
SET(MRPT_OPENGL ${MRPT_DIR}/opengl/include)
SET(MRPT_DIRS ${MRPT_BASE} ${MRPT_CONFIG} ${MRPT_BAYES} ${MRPT_OPENGL} ${MRPT_SC} ${MRPT_GUI} ${MRPT_OBS} ${MRPT_TOPO} ${MRPT_MAPS} ${MRPT_VISION} ${MRPT_HWD} ${MRPT_GRAPHS} ${MRPT_SLAM} ${MRPT_REACTIVE} ${MRPT_DETECTOR})
SET(MRPT_LIBS mrpt-base mrpt-bayes mrpt-opengl mrpt-scanmatching mrpt-gui mrpt-obs mrpt-topography mrpt-maps mrpt-vision mrpt-hwdrivers mrpt-slam mrpt-reactivenav mrpt-detectors)

# compile libzhpsm.so 
ADD_SUBDIRECTORY(zhpsm)
ADD_SUBDIRECTORY(mapGraph)
ADD_SUBDIRECTORY(timestamp)
ADD_SUBDIRECTORY(sick_reader)
ADD_SUBDIRECTORY(gps_reader)
ADD_SUBDIRECTORY(sensorFusion)
ADD_SUBDIRECTORY(ui)
ADD_SUBDIRECTORY(slam)
ADD_SUBDIRECTORY(ui_)

ADD_SUBDIRECTORY(rawseed)
ADD_SUBDIRECTORY(zhcsm)
ADD_SUBDIRECTORY(tcp_slam)
ADD_SUBDIRECTORY(version1)
ADD_SUBDIRECTORY(Main-Ctrl)
ADD_SUBDIRECTORY(zhicp)
ADD_SUBDIRECTORY(oneThread)
ADD_SUBDIRECTORY(drawmap)
ADD_SUBDIRECTORY(localization)
# ADD_SUBDIRECTORY(mrptpf)
