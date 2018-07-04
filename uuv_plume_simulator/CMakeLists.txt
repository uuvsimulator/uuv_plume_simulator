cmake_minimum_required(VERSION 2.8.3)
project(uuv_plume_simulator)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  visualization_msgs
  rospy
  uuv_plume_msgs
)

catkin_install_python(PROGRAMS scripts/plume_server
                               scripts/set_turbulent_plume
                               scripts/load_plume_particles
                               scripts/current_velocity_server
                               scripts/set_demo_turbulent_plume
                               scripts/set_demo_current_vel
                               scripts/set_demo_current_vel_gazebo
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
