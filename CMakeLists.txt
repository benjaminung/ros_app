project(CFE_ROS_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(../vec_app/fsw/platform_inc)

# Create the app module
add_cfe_app(ros_app fsw/src/ros_app.c)

# Include the public API from ros_lib to demonstrate how
# to call library-provided functions
add_cfe_app_dependency(ros_app ros_lib)