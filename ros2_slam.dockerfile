FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies 
RUN apt-get update && apt-get install -y \
    git wget curl sudo vim \
    locales gnupg2 lsb-release \
    libssl-dev libusb-1.0-0-dev \
    libudev-dev pkg-config \
    libgtk-3-dev cmake build-essential \
    python3-pip \
    screen && \
    locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ROS2 Humble sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list


RUN for i in 1 2 3 ; do \
        apt-get update && break || sleep 5 ; \
    done


# Install ROS2-specific packages (including colcon extensions)
RUN apt-get install -y --no-install-recommends \
        ros-humble-ros-base \   
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-tf2-tools \
        ros-humble-rviz2 \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*


RUN rosdep init || true && rosdep update


# librealsense setup
WORKDIR /opt
RUN git clone https://github.com/IntelRealSense/librealsense.git -b v2.53.1
WORKDIR /opt/librealsense
RUN mkdir build && cd build && \
    cmake ../ -DBUILD_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release -DBUILD_GRAPHICAL_EXAMPLES=false && \
    make -j$(nproc) && make install

RUN mkdir -p /etc/udev/rules.d && \
    ./scripts/setup_udev_rules.sh




# Create ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Intel RealSense ROS wrapper
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b 3.2.3

# Copy custom sources for RealSense 
COPY src/ realsense-ros/realsense2_camera/src/
COPY include/ realsense-ros/realsense2_camera/include/
COPY CMakeLists_docker.txt realsense-ros/realsense2_camera/CMakeLists.txt
COPY config/ /ros2_ws/config/

# urg_node2 setup
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git
RUN sed -i "s/params_ether.yaml/params_serial.yaml/" urg_node2/launch/urg_node2.launch.py

# Install the package dependencies of the workspace with rosdep
ENV ROS_DISTRO=humble

WORKDIR /ros2_ws

# (a) run rosdep, but ignore librealsense2 *and* cv_bridge
RUN for i in 1 2 3; do apt-get update && break || sleep 5 ; done && \
    rosdep install --from-paths src --ignore-src -r -y \
        --rosdistro=${ROS_DISTRO} \
        --skip-keys="librealsense2 cv_bridge"

# (b) dedicated install of the two big debs rosdep skipped
RUN for i in 1 2 3; do \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            ros-humble-cv-bridge \
            python3-opencv \
        && break || sleep 5 ; \
    done && \
    rm -rf /var/lib/apt/lists/*


# Build ROS2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Source setup files automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc


# Entry point setup
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
