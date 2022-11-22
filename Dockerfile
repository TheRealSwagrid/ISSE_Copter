FROM ros:noetic
SHELL ["/bin/bash", "-c"]

ENV semantix_port=7500

# ROS-Noetic Setup
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update
RUN apt-get update && apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential ros-noetic-mavros python3-rosdep python3-catkin-tools ros-noetic-vrpn-client-ros
#Install required Mavros packages, no included in standard install
RUN apt-get update && apt-get install -y ros-noetic-angles ros-noetic-tf ros-noetic-urdf ros-noetic-control-toolbox
RUN apt-get update && apt-get install -y python-is-python3 python3-pip git iputils-ping
RUN pip install playsound future pyyaml
RUN sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
#RUN sudo /ros_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
RUN rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# Add Files
ADD ros_ws /ros_ws
COPY protocols /etc
COPY ISSE_Copter.py ros_ws/src/rospkg
COPY AbstractVirtualCapability.py ros_ws/src/rospkg
COPY requirements /var

# Build Ros-Pkg and build
RUN cd /ros_ws && source /opt/ros/noetic/setup.bash && catkin build
#RUN source /ros_ws/devel/setup.bash

#Setup Env
#EXPOSE 9999 3883 11311 14555 14550
ENTRYPOINT ["/ros_entrypoint.sh"]
#RUN apt-get update && apt-get install -y  screen

#Start Copter
CMD source /ros_ws/devel/setup.bash && roslaunch rospkg copter.launch mav_id:=75 semantix_port:=${semantix_port}
#${semantix_port}
#CMD bash
