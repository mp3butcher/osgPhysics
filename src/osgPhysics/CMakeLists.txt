SET(LIB_NAME osgPhysics)

SET(HEADER_PATH ${PROJECT_SOURCE_DIR}/include/${LIB_NAME})
SET(HEADERS
    ${HEADER_PATH}/Export
    ${HEADER_PATH}/Version
    ${HEADER_PATH}/Utilities
    ${HEADER_PATH}/ComputeTrianglesVisitor
    ${HEADER_PATH}/ApplyVerticesVisitor
    ${HEADER_PATH}/PhysicsAttribute
    ${HEADER_PATH}/PhysicsThread
    ${HEADER_PATH}/AbstractInterface
    ${HEADER_PATH}/Joint
    ${HEADER_PATH}/BaseElement
    ${HEADER_PATH}/World
    ${HEADER_PATH}/RigidElement
    ${HEADER_PATH}/SoftBodyElement
    ${HEADER_PATH}/ClothElement

)

SET(SOURCES
    Version.cpp
    Utilities.cpp
    ComputeTrianglesVisitor.cpp
    ApplyVerticesVisitor.cpp
    PhysicsAttribute.cpp
    PhysicsThread.cpp
    AbstractInterface.cpp
    Joint.cpp
    BaseElement.cpp
    World.cpp
    RigidElement.cpp
    SoftBodyElement.cpp
    ClothElement.cpp

)

ADD_DEFINITIONS(-DOSGPHYSICS_LIBRARY)
ADD_LIBRARY(${LIB_NAME} SHARED ${HEADERS} ${SOURCES})
SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES DEBUG_POSTFIX ${OSG_DEBUG_POSTFIX})
TARGET_LINK_LIBRARIES(${LIB_NAME}
    debug OpenThreads${OSG_DEBUG_POSTFIX} optimized OpenThreads
    debug osg${OSG_DEBUG_POSTFIX}         optimized osg
    debug osgDB${OSG_DEBUG_POSTFIX}       optimized osgDB
    debug osgUtil${OSG_DEBUG_POSTFIX}     optimized osgUtil
)

SET(INSTALL_INCDIR include)
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

INSTALL(
    FILES       ${HEADERS}
    DESTINATION ${INSTALL_INCDIR}/${LIB_NAME}
)
