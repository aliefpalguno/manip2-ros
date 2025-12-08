cmake_minimum_required(VERSION 3.0.2)
project(manip2_control)

# 1. Find all required packages
find_package(catkin REQUIRED COMPONENTS
  rospy                  # Base for Python nodes
  roscpp                 # Base for C++ nodes (NEW)
  std_msgs               # Standard ROS messages
  sensor_msgs            # JointState
  std_srvs               # Trigger
  robot_state_publisher  # For URDF
  dynamixel_sdk          # For C++ Dynamixel bridge (NEW)
  manip2_msgs            # For ManipTelemetry, etc. (NEW)
)

# 2. Export dependencies
catkin_package(
  # List all packages your C++ and Python nodes depend on
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs std_srvs dynamixel_sdk manip2_msgs robot_state_publisher
)

# 3. Include directories for C++ compilation
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 4. C++ EXECUTABLE DEFINITION (for the converted node)
add_executable(manip2_dxl_bridge_telem scripts/manip2_dxl_bridge_telem.cpp)

# 5. Link libraries for the C++ executable
target_link_libraries(manip2_dxl_bridge_telem
  ${catkin_LIBRARIES}
)

# 6. Add dependencies (ensures custom messages are built before compiling C++)
add_dependencies(manip2_dxl_bridge_telem 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 7. INSTALL ALL PYTHON SCRIPTS
# Update this section to include all your Python nodes:
catkin_install_python(PROGRAMS
  scripts/manip2_dxl_bridge_telem.py  # (The original Python version)
  scripts/manip2_command_gateway.py
  scripts/manip2_matlab_bridge.py
  scripts/manip2_safety_supervisor.py
  scripts/manip2_web_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)