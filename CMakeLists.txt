install(DIRECTORY model   DESTINATION ${CNOID_SHARE_SUBDIR})
install(DIRECTORY project DESTINATION ${CNOID_SHARE_SUBDIR})
install(DIRECTORY motion  DESTINATION ${CNOID_SHARE_SUBDIR})

add_subdirectory(src)
add_subdirectory(controller)
