# Download base image ROS2 Humble Hawksbill
FROM osrf/ros:humble-desktop

# Arguments for building
ARG USERID
ARG USER
ARG SSH_PRV_KEY

# Image name
ARG "Base for ROS2 Humble Hawksbill development."

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/install_deps.sh .
COPY docker/requirements.txt .
COPY docker/requirements_ros.txt .
COPY docker/requirements_python.txt .
RUN ./install_deps.sh
RUN rm install_deps.sh requirements.txt requirements_ros.txt requirements_python.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "InOrbit developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Run rosdep update.
RUN rosdep update

# Creates the src folder of the workspace
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros2 humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc

# More ROS2 settings
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$USER/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/$USER/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
