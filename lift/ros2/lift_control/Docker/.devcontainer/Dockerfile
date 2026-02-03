# Use ROS2 Humble base image for ARM64
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=30

# Install development dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    gdb \
    clang-format \
    clang-tidy \
    valgrind \
    vim \
    nano \
    wget \
    curl \
    htop \
    tmux \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

#  Install WiringPi manually from source (replace wiringpi_3.16_arm64.deb with appropriate version if needed)
RUN git clone https://github.com/WiringPi/WiringPi.git /tmp/WiringPi && \
    cd /tmp/WiringPi && \
    ./build debian && \
    mv debian-template/wiringpi_3.16_arm64.deb /tmp/wiringpi.deb && \
    apt-get update && \
    apt install -y /tmp/wiringpi.deb && \
    rm -rf /tmp/WiringPi /tmp/wiringpi.deb && \
    rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/lift_ws

# Install Python development tools
RUN pip3 install --no-cache-dir \
    pylint \
    black \
    flake8 \
    mypy

# Setup ROS2 environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias ct='colcon test'" >> ~/.bashrc && \
    echo "alias ctr='colcon test-result --verbose'" >> ~/.bashrc && \
    echo "alias source_ws='source /root/lift_ws/install/setup.bash'" >> ~/.bashrc

# Enable bash completion for ROS2
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc

# Keep container running
CMD ["/bin/bash"]



