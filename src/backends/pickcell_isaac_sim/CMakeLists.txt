cmake_minimum_required(VERSION 3.8)
project(pickcell_isaac_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/run_pickcell_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
