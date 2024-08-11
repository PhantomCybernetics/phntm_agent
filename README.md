`sudo usermod -aG docker ${USER}`
Log in and out

`ROS_DISTRO=humble; \
`docker build -f Dockerfile -t phntm/agent:$ROS_DISTRO \
`  --build-arg ROS_DISTRO=$ROS_DISTRO \
`  --build-arg ARCH=aarch64 \
` .