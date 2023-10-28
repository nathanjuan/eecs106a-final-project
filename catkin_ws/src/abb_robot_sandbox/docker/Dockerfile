FROM osrf/ros:melodic-desktop-full

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update && apt-get install -y \
    sudo \
    python-catkin-tools \
    ros-melodic-joint-state-publisher-gui \
    ros-melodic-kdl-parser-py \
    ros-melodic-industrial-core \
    ros-melodic-abb-rapid-msgs \
    ros-melodic-abb-egm-msgs \
    ros-melodic-velocity-controllers \
    ros-melodic-abb-rapid-sm-addin-msgs \
    ros-melodic-abb-robot-msgs \
    ros-melodic-abb-egm-msgs \
    ros-melodic-abb-rapid-msgs \
    ros-melodic-abb-rapid-sm-addin-msgs \
    ros-melodic-abb-robot-msgs \
    ros-melodic-moveit-fake-controller-manager \
    ros-melodic-velocity-controllers \
    ros-melodic-abb-driver \
    ros-melodic-moveit-planners-ompl \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-ros-controllers \
    ros-melodic-moveit-kinematics \
    ros-melodic-moveit-simple-controller-manager \
    ros-melodic-moveit-ros-visualization \
    ros-melodic-trac-ik-kinematics-plugin \
    ros-melodic-soem \
    ros-melodic-socketcan-interface \
    xsltproc \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    protobuf-compiler \
	nano \
  && rm -rf /var/lib/apt/lists/*

ENV USERNAME oski
RUN mkdir -p /home/$USERNAME/catkin_ws/src && \
    echo "$USERNAME:x:1000:1000:Developer,,,:/home/$USERNAME:/bin/bash" >> /etc/passwd && \
    echo "$USERNAME:x:1000:" >> /etc/group && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    chown 1000:1000 -R /home/$USERNAME

USER $USERNAME
ENV HOME /home/$USERNAME

COPY entrypoint.sh /entrypoint.sh

RUN apt-get update -qq && rosdep update && rosdep install --from-paths /home/$USERNAME/catkin_ws/src --ignore-src -y
