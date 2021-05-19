FROM ryuichiueda/ubuntu18.04-ros-image
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update
RUN apt-get install -y ros-melodic-desktop-full 
RUN apt-get install -y ros-melodic-tf ros-melodic-tf2 ros-melodic-tf2-geometry-msgs ros-melodic-urdf ros-melodic-map-server xvfb vim psmisc ros-melodic-move-base* ros-melodic-dwa-local-planner ros-melodic-global-planner ros-melodic-grid-map

RUN cd /catkin_ws/src && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin.git && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN source ~/.bashrc && \
    cd /catkin_ws && \
    catkin_make
