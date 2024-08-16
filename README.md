`sudo usermod -aG docker ${USER}`
Log in and out

`ROS_DISTRO=humble; \
`docker build -f Dockerfile -t phntm/agent:$ROS_DISTRO \
`  --build-arg ROS_DISTRO=$ROS_DISTRO \
`  --build-arg ARCH=aarch64 \
` .

intit symlinked fs:
`cd /ros2_ws`
`colcon build --symlink-install --packages-select phntm_agent`

`ros2 launch phntm_agent agent_launch.py`