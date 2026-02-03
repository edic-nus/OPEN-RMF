# Use ROS2 Humble base image for ARM64
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=30

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    gettext-base \
    fakeroot \
    && rm -rf /var/lib/apt/lists/*

# Install WiringPi manually from source
RUN git clone https://github.com/WiringPi/WiringPi.git /tmp/WiringPi && \
    cd /tmp/WiringPi && \
    ./build debian && \
    mv debian-template/wiringpi_*.deb /tmp/wiringpi.deb && \
    apt-get update && \
    apt install -y /tmp/wiringpi.deb && \
    rm -rf /tmp/WiringPi /tmp/wiringpi.deb && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/lift_ws

# Copy your package into the container
COPY . /root/lift_ws/src/lift_control/

# Install rosdep dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Source the workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/lift_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc

# Set up entrypoint
COPY docker-entrypoint.sh /root/
RUN chmod +x /root/docker-entrypoint.sh

ENTRYPOINT ["/root/docker-entrypoint.sh"]
CMD ["ros2", "run", "lift_control", "lift_rpi"]