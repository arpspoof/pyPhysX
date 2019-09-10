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

set(CORE_PATH "${SRC_PATH}/core")
set(ARTICULATION_PATH "${SRC_PATH}/articulation")

set(RENDERER_PATH "${SRC_PATH}/renderer")
set(RENDERER_GLUT_PATH "${RENDERER_PATH}/glut")

include_directories("${PHYSX_INSTALL_DIR}/PhysX/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/foundation/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/fastxml/include")
include_directories("${PHYSX_INSTALL_DIR}/PxShared/include")
include_directories("${EIGEN_DIR}")

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


file(GLOB CORE_SRC
    "${CORE_PATH}/include/*.h"
    "${CORE_PATH}/*.cpp"
)
add_library(core ${CORE_SRC})
target_link_libraries(core ${PX_LIBS} ${PX_LIBS} ${SYS_LIBS})


file(GLOB ARTICULATION_SRC
    "${ARTICULATION_PATH}/include/*.h"
    "${ARTICULATION_PATH}/*.cpp"
)
add_library(articulation ${ARTICULATION_SRC})
target_link_libraries(articulation ${PX_LIBS} ${PX_LIBS} ${SYS_LIBS})


file(GLOB RENDERER_GLUT_SRC
    "${RENDERER_GLUT_PATH}/include/*.h"
    "${RENDERER_GLUT_PATH}/*.cpp"
)
add_library(renderer_glut ${RENDERER_GLUT_SRC})
target_link_libraries(renderer_glut ${PX_LIBS} ${PX_LIBS} ${SYS_LIBS})


add_executable(main "${PROJECT_SOURCE_DIR}/src/main.cpp")
target_link_libraries(main core articulation renderer_glut)
