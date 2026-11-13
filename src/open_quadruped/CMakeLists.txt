cmake_minimum_required(VERSION 3.8)
project(open_quadruped)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointAngles.msg"
)

ament_export_dependencies(rosidl_default_runtime)

install(PROGRAMS
  nodes/interface_process.py
  nodes/joint_angles_to_trajectory.py
  nodes/keyboard_foot_control.py
  nodes/crawl_gait_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
