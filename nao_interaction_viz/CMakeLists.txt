cmake_minimum_required(VERSION 2.8.3)
project(nao_interaction_viz)

#Declare build dependencies
find_package(catkin 
    COMPONENTS
    nao_interaction_msgs
    rviz)

#Declare package run-time dependencies
catkin_package()

add_subdirectory(src)

# install
install(FILES person_rviz_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
