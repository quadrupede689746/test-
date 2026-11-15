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
  nodes/trajectory_to_effort.py
  nodes/enable_robot_gravity.py
  nodes/keyboard_walk.py
  nodes/robot_logger.py
  nodes/analyze_log.py
  nodes/single_foot_test.py
  nodes/stance_bend_test.py
  nodes/place_bent_stance.py
  nodes/keyboard_foot_control.py
  nodes/crawl_gait_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  nodes/kinematics.py
  nodes/gait_generator.py
  nodes/balance.py
  nodes/safety_monitor.py
  nodes/keyboard_input.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(motion_modules test/test_motion_modules.py)
endif()

ament_package()
