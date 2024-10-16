include_directories("../../src")
include_directories("${DYMP_MPC_VNOID_DIR}/src")

add_cnoid_simple_controller(dymp_mpc_dance_controller
  main.cpp
  poseseq.yaml
  planner_wholebody.yaml
  planner_centroid.yaml
  kinematics.yaml
  robot.yaml
  )

target_include_directories(dymp_mpc_dance_controller PUBLIC ${MKL_INCLUDE_DIR})
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_sequential.lib)

target_link_libraries(dymp_mpc_dance_controller
 dymp_mpc_lib
)
