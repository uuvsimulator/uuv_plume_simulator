cmake_minimum_required(VERSION 2.8.3)
project(uuv_plume_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    geographic_msgs
    message_generation
)

add_message_files(
  FILES
  ParticleConcentration.msg
  Salinity.msg
)

add_service_files(
  FILES
  CreatePassiveScalarTurbulentPlume.srv
  CreateSpheroidPlume.srv
  SetPlumeLimits.srv
  SetPlumeConfiguration.srv
  GetPlumeConfiguration.srv
  DeletePlume.srv
  SetPlumeSourcePosition.srv
  GetPlumeSourcePosition.srv
  GetNumParticles.srv
  StorePlumeState.srv
  LoadPlumeParticles.srv
  SetCurrentDirection.srv
  SetCurrentModel.srv
  GetCurrentModel.srv
  SetCurrentVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)


catkin_package(
    CATKIN_DEPENDS
      std_msgs
      geometry_msgs
      geographic_msgs
      message_runtime
)
