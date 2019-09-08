cmake_minimum_required(VERSION 3.15)

set (CMAKE_CXX_STANDARD 11)

project(pyPhysX)

if (NOT DEFINED PHYSX_INSTALL_DIR)
    message(FATAL_ERROR "No PhysX install directory specified")
endif()

if (NOT DEFINED EIGEN_DIR)
    message(FATAL_ERROR "No Eigen directory specified")
endif()

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/foundation/include")
include_directories("${PHYSX_INSTALL_DIR}/PhysX/source/fastxml/include")
include_directories("${PHYSX_INSTALL_DIR}/PxShared/include")
include_directories("${EIGEN_DIR}")

# This should be fixed: linux.clang
link_directories("${PHYSX_INSTALL_DIR}/PhysX/bin/linux.clang/release")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    add_compile_definitions(_DEBUG)
else()
    add_compile_definitions(NDEBUG)
endif()

file(GLOB SRC
    "${PROJECT_SOURCE_DIR}/src/*.h"
    "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(main ${SRC})

target_link_libraries(main 
libPhysX_static_64.a 
libPhysXCommon_static_64.a
libPhysXCharacterKinematic_static_64.a
libPhysXFoundation_static_64.a
libPhysXPvdSDK_static_64.a
libPhysXCooking_static_64.a
libPhysXExtensions_static_64.a
libPhysXVehicle_static_64.a

libPhysX_static_64.a 
libPhysXCommon_static_64.a
libPhysXCharacterKinematic_static_64.a
libPhysXFoundation_static_64.a
libPhysXPvdSDK_static_64.a
libPhysXCooking_static_64.a
libPhysXExtensions_static_64.a
libPhysXVehicle_static_64.a

dl
pthread
GL
GLU
glut
X11
rt
cuda

)
