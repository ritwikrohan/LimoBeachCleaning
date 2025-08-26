cmake_minimum_required(VERSION 3.10.2)
project(robotiq_85_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
