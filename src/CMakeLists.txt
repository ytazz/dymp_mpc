
# the following line lets cmake to use FindMKL.cmake in this source directory
# if FindMKL fails to identify correct mkl, specify MKL_ROOT variable directly
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")

find_package(yaml-cpp)
find_package(OpenMP)
#find_package(GLEW)
find_package(dymp)
find_package(glfw3)
find_package(Eigen3 REQUIRED)
find_package(MKL    REQUIRED)

# to include vnoid headers
set(DYMP_MPC_VNOID_DIR "" CACHE PATH "top directory of vnoid")
include_directories("${DYMP_MPC_VNOID_DIR}/src")

set(DYMP_MPC_HEADERS
	"base.h"
	"criticalsection.h"
    "event.h"
    "kinematics.h"
    "myrobot.h"
    "planner.h"
    "planner_centroid.h"
    "planner_wholebody.h"
	"poseseq.h"
	"thread.h"
	)
set(DYMP_MPC_SOURCES
	"criticalsection.cpp"
    "event.cpp"
    "kinematics.cpp"
	"myrobot.cpp"
    "planner.cpp"
    "planner_centroid.cpp"
    "planner_wholebody.cpp"
	"poseseq.cpp"
	"thread.cpp"
	)

choreonoid_add_library(dymp_mpc_lib STATIC ${DYMP_MPC_HEADERS} ${DYMP_MPC_SOURCES})

target_include_directories(dymp_mpc_lib PUBLIC ${MKL_INCLUDE_DIR})
target_link_libraries(dymp_mpc_lib
    PRIVATE ${MKL_LIBRARIES}
)

#target_link_libraries(dymp_mpc_lib ${MKL_LIB_DIR}/mkl_core.lib)
#target_link_libraries(dymp_mpc_lib ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
#target_link_libraries(dymp_mpc_lib ${MKL_LIB_DIR}/mkl_sequential.lib)

target_link_libraries(dymp_mpc_lib PRIVATE
 vnoid_lib
 yaml-cpp::yaml-cpp
 Eigen3::Eigen
 dymp
)


