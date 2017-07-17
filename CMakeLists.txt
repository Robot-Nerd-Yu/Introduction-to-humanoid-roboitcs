cmake_minimum_required(VERSION 2.8.3)
project(introduction-to-humanoid-robotics)

find_package(catkin REQUIRED COMPONENTS
  openrave_catkin
  rospy
)

# catkin_python_setup()

catkin_package()

install(DIRECTORY objects robots worlds
        DESTINATION ${OpenRAVE_DEVEL_DIR}/${OpenRAVE_DATA_DIR})
install(DIRECTORY objects robots worlds
        DESTINATION ${OpenRAVE_INSTALL_DIR}/${OpenRAVE_DATA_DIR})