FROM osrf/ros:jazzy-desktop-full

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages required for VisionBot
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # Robot description and visualization
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-rviz2 \
    # ROS 2 Control
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-gz-ros2-control \
    # Gazebo integration
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    # Mobile app communication
    ros-jazzy-rosbridge-server \
    ros-jazzy-twist-stamper \
    # Utilities
    nano \
    vim \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (if not already done)
RUN rosdep update || true

COPY src/vision_bot/config/ /site_config/

# The base image already has user 'ubuntu' with UID 1000
# Add ubuntu user to sudo group (optional, for development)
RUN apt-get update && apt-get install -y sudo \
    && echo "ubuntu ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace directory
RUN mkdir -p /home/ubuntu/ros2_ws/src && chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Switch to non-root user
USER ubuntu
WORKDIR /home/ubuntu/ros2_ws

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
