cmake_minimum_required(VERSION 3.0.2)
project(lar_fusao_ekf)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gt_odom.py
  scripts/fusion_error.py
  scripts/drive_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
