FROM osrf/ros:melodic-desktop-full

SHELL ["/bin/bash", "-c"]

COPY ISSE_Copter.py /var
COPY AbstractVirtualCapability.py /var
COPY requirements /var

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-get update && apt-get install -y curl
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
#RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update
RUN apt-get update && apt-get install -y ros-melodic-desktop-full ros-melodic-mavros python-rosinstall python-rosinstall-generator python-wstool build-essential ros-melodic-mavros-extras ros-melodic-vrpn-client-ros ros-melodic-catkin
RUN apt-get update && apt-get install -y python-pip git
RUN pip install playsound
RUN sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
RUN rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN mkdir -p ~/catkin_ws/src

RUN cd ~/catkin_ws/src && git clone https://alfanolu:IgzUA123!@gitlab.isse.de/robotik/semanticplugandplay/roscopter.git -

RUN source /opt/ros/melodic/setup.bash && cd ~/catkin_ws &&catkin_make
RUN source ~/catkin_ws/devel/setup.bash

EXPOSE 9999
RUN python3 /var/ISSE_Copter.py
CMD bash