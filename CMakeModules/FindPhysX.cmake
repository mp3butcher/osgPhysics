FIND_PATH(PHYSX_SDK_ROOT SDKs/PhysXLoader/include/PhysXLoader.h
    PATHS
    $ENV{PATH}
    /usr/include/
    /usr/local/include/
)

FIND_PATH(PHYSX_LOADER_INCLUDE_DIR PhysXLoader.h
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs
    /usr/include/${PHYSX_SDK_ROOT}/SDKs
    /usr/local/include/${PHYSX_SDK_ROOT}/SDKs
    PATH_SUFFIXES
    PhysXLoader/include
)

FIND_PATH(PHYSX_PHYSICS_INCLUDE_DIR NxPhysicsSDK.h
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs
    /usr/include/${PHYSX_SDK_ROOT}/SDKs
    /usr/local/include/${PHYSX_SDK_ROOT}/SDKs
    PATH_SUFFIXES
    Physics/include
)

FIND_PATH(PHYSX_FOUNDATION_INCLUDE_DIR NxFoundation.h
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs
    /usr/include/${PHYSX_SDK_ROOT}/SDKs
    /usr/local/include/${PHYSX_SDK_ROOT}/SDKs
    PATH_SUFFIXES
    Foundation/include
)

FIND_PATH(PHYSX_COOKING_INCLUDE_DIR NxCooking.h
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs
    /usr/include/${PHYSX_SDK_ROOT}/SDKs
    /usr/local/include/${PHYSX_SDK_ROOT}/SDKs
    PATH_SUFFIXES
    Cooking/include
)

FIND_PATH(PHYSX_CHARACTER_INCLUDE_DIR NxCharacter.h
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs
    /usr/include/${PHYSX_SDK_ROOT}/SDKs
    /usr/local/include/${PHYSX_SDK_ROOT}/SDKs
    PATH_SUFFIXES
    NxCharacter/include
)

FIND_LIBRARY(PHYSX_LOADER_LIB
    NAMES PhysXLoader.lib libPhysXLoader.so
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs/lib/Win32
    /usr/lib/${PHYSX_SDK_ROOT}
    /usr/local/lib/${PHYSX_SDK_ROOT}
)

FIND_LIBRARY(PHYSX_COOKING_LIB
    NAMES NxCooking.lib PhysXCooking.lib libNxCooking.so libPhysXCooking.so
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs/lib/Win32
    /usr/lib/${PHYSX_SDK_ROOT}
    /usr/local/lib/${PHYSX_SDK_ROOT}
)

FIND_LIBRARY(PHYSX_CHARACTER_LIB
    NAMES NxCharacter.lib libNxCharacter.so
    PATHS
    ${PHYSX_SDK_ROOT}/SDKs/lib/Win32
    /usr/lib/${PHYSX_SDK_ROOT}
    /usr/local/lib/${PHYSX_SDK_ROOT}
)

SET(PHYSX_SDK_FOUND "NO")
IF(PHYSX_LOADER_INCLUDE_DIR AND PHYSX_LOADER_LIB)
    SET(PHYSX_SDK_FOUND "YES")
ENDIF(PHYSX_LOADER_INCLUDE_DIR AND PHYSX_LOADER_LIB)

SET(PHYSX_COOKING_FOUND "NO")
IF(PHYSX_COOKING_INCLUDE_DIR AND PHYSX_COOKING_LIB)
    SET(PHYSX_COOKING_FOUND "YES")
ENDIF(PHYSX_COOKING_INCLUDE_DIR AND PHYSX_COOKING_LIB)

SET(PHYSX_CHARACTER_FOUND "NO")
IF(PHYSX_CHARACTER_INCLUDE_DIR AND PHYSX_CHARACTER_LIB)
    SET(PHYSX_CHARACTER_FOUND "YES")
ENDIF(PHYSX_CHARACTER_INCLUDE_DIR AND PHYSX_CHARACTER_LIB)

MARK_AS_ADVANCED(PHYSX_LOADER_INCLUDE_DIR PHYSX_COOKING_INCLUDE_DIR PHYSX_CHARACTER_INCLUDE_DIR
                 PHYSX_LOADER_LIB PHYSX_COOKING_LIB PHYSX_CHARACTER_LIB
)
