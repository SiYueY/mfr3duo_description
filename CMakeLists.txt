cmake_minimum_required(VERSION 3.16)
project(mfr3duo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/visualize_urdf.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualize_urdf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/test_urdf.py)
endif()

ament_package()
