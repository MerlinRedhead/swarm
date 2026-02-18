cmake_minimum_required(VERSION 3.8)
project(swarm_formation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionState.msg"
  "msg/AgentStatus.msg"
  "msg/Target.msg"
  "msg/AttackCommand.msg"
  "msg/GimbalState.msg"
  "msg/GuiOverlay.msg"
  "msg/SwarmCommand.msg"
  "msg/TargetRefined.msg"
  DEPENDENCIES geometry_msgs
)

# Установка Python скриптов
install(PROGRAMS
  swarm_formation/hub_commander.py
  swarm_formation/formation_member.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
