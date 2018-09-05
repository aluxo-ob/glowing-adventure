cmake_minimum_required(VERSION 2.8.3)
project(navigans_msgs)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  message_generation
  actionlib_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  )

#add_message_files(
#  DIRECTORY msg
#  FILES
#  PlanEnvelope.msg
#  ConstraintStamped.msg
#  ConstraintList.msg
#  Status.msg
#  RoadmapEdge.msg
#  Roadmap.msg
#  )

# actions
add_action_files(
  DIRECTORY action
  FILES
#  ControlToPlan.action
#  ComputeLocalPlan.action
#  ComputePlan.action
#  VerifyPlan.action
  ExternalPath.action
#  GotoRegion.action
#  SearchPatrol.action
  )



generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs std_msgs geometry_msgs nav_msgs
  )

set(${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_gencpp)
