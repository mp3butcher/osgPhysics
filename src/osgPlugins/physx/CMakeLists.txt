SET(LIB_NAME osgdb_physx)

SET(SOURCES
	ReaderWriterPhysX.cpp
	PhysXInterface.cpp
	
	RigidImplementation.cpp
	SoftBodyImplementation.cpp
)

INCLUDE_DIRECTORIES(${PHYSX_LOADER_INCLUDE_DIR}
                    ${PHYSX_PHYSICS_INCLUDE_DIR}
                    ${PHYSX_FOUNDATION_INCLUDE_DIR}
                    ${PHYSX_COOKING_INCLUDE_DIR}
)

ADD_LIBRARY(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES DEBUG_POSTFIX ${OSG_DEBUG_POSTFIX})
TARGET_LINK_LIBRARIES(${LIB_NAME}
    debug osg${OSG_DEBUG_POSTFIX}         optimized osg
    debug osgDB${OSG_DEBUG_POSTFIX}       optimized osgDB
    debug OpenThreads${OSG_DEBUG_POSTFIX} optimized OpenThreads
    debug osgUtil${OSG_DEBUG_POSTFIX}     optimized osgUtil
    
    ${PHYSX_LOADER_LIB}
    ${PHYSX_COOKING_LIB}
    
    osgPhysics
)

SET(INSTALL_BINDIR bin)
IF(WIN32)
    SET(INSTALL_LIBDIR bin)
    SET(INSTALL_ARCHIVEDIR lib)
ELSE(WIN32)
    SET(INSTALL_LIBDIR lib${LIB_POSTFIX})
    SET(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
ENDIF(WIN32)

INSTALL(
    TARGETS ${LIB_NAME}
    RUNTIME DESTINATION ${INSTALL_BINDIR}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
)
