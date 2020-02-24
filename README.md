# minmax

1) Install ROS melodic and catkin following this link: https://wiki.ros.org/catkin#Installing_catkin
2) $ git clone https://github.com/Silviatulli/minmax.git in the src folder of your catkin workspace
3) Build a catkin workspace and source the setup file run:
    $ cd ~/catkin_ws
    $ catkin_make
4) add the workspace to the ROS environment:
    . ~/catkin_ws/devel/setup.bash
5) Make sure that the CMakeLists.txt file is configured properly, with all the services and the dependencies listed as follows:
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      message_generation
      message_runtime
    )
    add_service_files(
       FILES
       Decision.srv
       GameState.srv
       Plan.srv
       RobotExplanation.srv
       RobotTalk.srv
     )

