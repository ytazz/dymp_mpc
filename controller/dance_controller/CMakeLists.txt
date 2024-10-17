include_directories("../../src")
include_directories("${DYMP_MPC_VNOID_DIR}/src")

add_cnoid_simple_controller(dymp_mpc_dance_controller
  main.cpp
  config/poseseq.yaml
  config/planner_wholebody.yaml
  config/planner_centroid.yaml
  config/kinematics.yaml
  config/robot.yaml
  )

target_include_directories(dymp_mpc_dance_controller PUBLIC ${MKL_INCLUDE_DIR})
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_core.lib)
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_intel_lp64.lib)
target_link_libraries(dymp_mpc_dance_controller ${MKL_LIB_DIR}/mkl_sequential.lib)

target_link_libraries(dymp_mpc_dance_controller
 dymp_mpc_lib
)

# copy config files
install(DIRECTORY config DESTINATION ${CNOID_SHARE_SUBDIR}/project/dymp_mpc/dance_controller)
