cmake_minimum_required(VERSION 3.5)
project(ros_components_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch config test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(pytest_tests
    test/test_components_xacro.py
  )
  foreach(test_path ${pytest_tests})
    get_filename_component(test_name ${test_path} NAME_WE)
    ament_add_pytest_test(${test_name} ${test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
