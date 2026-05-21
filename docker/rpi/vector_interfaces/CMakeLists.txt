cmake_minimum_required(VERSION 3.8)
project(vector_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SttResult.msg"
  "msg/RagQuery.msg"
  "msg/RagContext.msg"
  "msg/LLMToolCall.msg"
  "msg/RobotStats.msg"
  "msg/SystemStats.msg"
  "msg/BatteryStats.msg"
  "srv/NavigateToLocation.srv"
  "srv/SetLocation.srv"
  "srv/GetLocations.srv"
  "srv/DeleteLocation.srv"
  "srv/RenameLocation.srv"
  "srv/SetMode.srv"
  "srv/GetMode.srv"
  "srv/SaveMap.srv"
  "srv/ListMaps.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
