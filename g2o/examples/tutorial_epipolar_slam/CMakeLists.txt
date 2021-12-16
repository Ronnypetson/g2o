add_library(tutorial_epipolar_slam_library ${G2O_LIB_TYPE}
  se3.h se3.cpp
  g2o_tutorial_epipolar_slam_api.h
  vertex_se3.h  vertex_se3.cpp
)

set_target_properties(tutorial_epipolar_slam_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_epipolar_slam)

target_link_libraries(tutorial_epipolar_slam_library core solver_eigen)

add_executable(tutorial_epipolar_slam
  tutorial_epipolar_slam.cpp
)

set_target_properties(tutorial_epipolar_slam PROPERTIES OUTPUT_NAME tutorial_epipolar_slam${EXE_POSTFIX})

target_link_libraries(tutorial_epipolar_slam tutorial_epipolar_slam_library)
