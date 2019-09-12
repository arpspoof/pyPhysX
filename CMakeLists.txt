cmake_minimum_required(VERSION 3.15)

set (CMAKE_CXX_STANDARD 11)

project(pyPhysX)

# Example install directory for linux:
# ---/PhysX/physx/install/linux
if (NOT DEFINED PHYSX_INSTALL_DIR)
    message(FATAL_ERROR "No PhysX install directory specified")
endif()

# Example build directory for linux:
# ---/PhysX/physx/bin/linux.clang/release
if (NOT DEFINED PHYSX_BUILD_DIR)
    message(FATAL_ERROR "No PhysX build directory specified")
endif()

if (NOT DEFINED EIGEN_DIR)
    message(FATAL_ERROR "No Eigen directory specified")
endif()

set(SRC_PATH "${PROJECT_SOURCE_DIR}/src")

set(COMMON_PATH "${SRC_PATH}/common")
set(CORE_PATH "${SRC_PATH}/core")
set(ARTICULATION_PATH "${SRC_PATH}/articulation")

set(RENDERER_PATH "${SRC_PATH}/renderer")
set(RENDERER_GLUT_PATH "${RENDERER_PATH}/glut")

include_directories("${PHYSX_INSTALL_DIR}/PhysX/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/foundation/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/fastxml/include")
include_directories("${PHYSX_INSTALL_DIR}/PxShared/include")
include_directories("${EIGEN_DIR}")

include_directories("${COMMON_PATH}/include")
include_directories("${CORE_PATH}/include")
include_directories("${ARTICULATION_PATH}/include")
include_directories("${RENDERER_GLUT_PATH}/include")

link_directories("${PHYSX_BUILD_DIR}")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    add_compile_definitions(_DEBUG)
else()
    add_compile_definitions(NDEBUG)
endif()

set(PX_LIBS 
    libPhysX_static_64.a 
    libPhysXCommon_static_64.a
    libPhysXCharacterKinematic_static_64.a
    libPhysXFoundation_static_64.a
    libPhysXPvdSDK_static_64.a
    libPhysXCooking_static_64.a
    libPhysXExtensions_static_64.a
    libPhysXVehicle_static_64.a
)

set(SYS_LIBS
    dl
    pthread
    GL
    GLU
    glut
    X11
    rt
    cuda
)


function(createLibrary varName varSrcPath)
    file(GLOB TMP_SRC
        "${varSrcPath}/include/*.h"
        "${varSrcPath}/*.cpp"
    )
    add_library(${varName} ${TMP_SRC})
    target_link_libraries(${varName} ${ARGN})
endfunction(createLibrary)

function(createExecutable varName varSrcPath)
    file(GLOB TMP_SRC
        "${varSrcPath}/include/*.h"
        "${varSrcPath}/*.cpp"
    )
    add_executable(${varName} ${TMP_SRC})
    target_link_libraries(${varName} ${ARGN})
endfunction(createExecutable)

createLibrary(articulation ${ARTICULATION_PATH})
createLibrary(core ${CORE_PATH})
createLibrary(renderer_glut ${RENDERER_GLUT_PATH})

set(ALL_MODULE_NO_RENDERER articulation core)
set(ALL_MODULE ${ALL_MODULE_NO_RENDERER} renderer_glut)
set(ALL_EXT_LIBS ${PX_LIBS} ${PX_LIBS} ${SYS_LIBS})


set(BUILD_EXAMPLES true)

if(${BUILD_EXAMPLES} MATCHES true)

    set(EXAMPLE_PATH "${SRC_PATH}/examples")

    set(EXAMPLE_POSE_TRACKING_PATH "${EXAMPLE_PATH}/poseTracking")
    createExecutable(poseTracking ${EXAMPLE_POSE_TRACKING_PATH} ${ALL_MODULE} ${ALL_EXT_LIBS})

endif()



set(BUILD_TESTS true)

if (${BUILD_TESTS} MATCHES true)

    find_package(GTest REQUIRED)
    include_directories(${GTEST_INCLUDE_DIRS})

    set(TESTS_PATH ${SRC_PATH}/tests)
 #   createExecutable(unitTests ${TESTS_PATH} ${ALL_MODULE_NO_RENDERER} ${ALL_EXT_LIBS} ${GTEST_LIBRARIES})
    createExecutable(unitTests ${TESTS_PATH} ${ALL_MODULE} ${ALL_EXT_LIBS} ${GTEST_LIBRARIES})

endif()
