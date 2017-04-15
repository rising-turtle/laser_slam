
# find CSM libs
INCLUDE_DIRECTORIES(${CSM_INC_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../zhpsm ${EIGEN})
LINK_DIRECTORIES(${CSM_LIB} ${CMAKE_BINARY_DIR}/lib)

SET(LIB_LIST
	ZHCanonical_Matcher.cpp
)
SET(CSM_LIBS csm csm-static egsl options)

# out lib path
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
ADD_LIBRARY(zhcsm SHARED ${LIB_LIST})

ADD_EXECUTABLE(test_csm test.cpp)
TARGET_LINK_LIBRARIES(test_csm zhcsm zhpsm ${CSM_LIBS})


