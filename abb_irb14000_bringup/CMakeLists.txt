cmake_minimum_required(VERSION 2.8.3)
project(abb_irb14000_bringup)

find_package(catkin 
  REQUIRED COMPONENTS
    roslaunch  
)

catkin_package()

roslaunch_add_file_check(launch/arms_hw_interface.launch)
roslaunch_add_file_check(launch/grippers.launch)
roslaunch_add_file_check(launch/controllers.launch)
roslaunch_add_file_check(launch/view_live_robot.launch)