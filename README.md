# Phantom Bridge Agent

This is a supporting package for the [Phantom Brige Client](https://github.com/PhantomCybernetics/phntm_bridge_client).

The Agent monitors system resources (such as memory, disk usage and CPU load), Wi-Fi connection quality (allowing to scan and roam between APs), allows to control configured Docker containers and to monitor their used resources.

It also enables extraction of files from any running Docker container, identified either with absolute path, file:// or ROS2 package:// format. Upon receiving file request from the Client node, Agent queries all observed running Docker containers. If the requested file is found, it either uploads it to the Bridge Server directly, or splits it into chunks to be uploaded by another Agent instace.

Typically, the Agent is installed with the Phantom Bridge Client and runs inside its Docker container, in which case it can also share one config file with the Client.
However, it can be also installed in a standalone mode, which is useful for monitoring and control of distributed ROS2 systems. In this case, only one Agent node needs to have internet access for file extraction to work.

![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fhumble-amd64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Firon-amd64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fjazzy-amd64.json)  ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fkilted-amd64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Flyrical-amd64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Frolling-amd64.json) \
![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fhumble-arm64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Firon-arm64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fjazzy-arm64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Fkilted-arm64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Flyrical-arm64.json) ![Endpoint Badge](https://img.shields.io/endpoint?url=https%3A%2F%2Fphantomcybernetics.github.io%2Fphntm_agent%2Frolling-arm64.json)

## Architecture
![Infrastructure map](https://raw.githubusercontent.com/PhantomCybernetics/phntm_bridge_docs/refs/heads/main/img/Architecture_Agent.svg)

## Standalone Install

### Make sure your root SSL Certificates are up to date

```bash
sudo apt update
sudo apt install ca-certificates
```

### Install Docker, Docker Build & Docker Compose

E.g. on Debian/Ubuntu follow [these instructions](https://docs.docker.com/engine/install/debian/). Then add the current user to the docker group:
```bash
sudo usermod -aG docker ${USER}
# log out & back in
```

### Configure the Agent
Here's an example config file, e.g. `~/phntm_agent.yaml`. The full list of configuration options can be found [here](https://docs.phntm.io/bridge/basics/configuration#agent-configuration).
```yaml
/**:
  ros__parameters:

    host_name: 'pi5' # lower case, must be a valid ROS id or ''
    agent_update_period_sec: 0.5

    docker_monitor_topic: '/docker_info' # '' to disable
    enable_docker_control: True # allow start/stop/restart of a container

    system_info_topic: '/system_info_pi5' # writes output here, '' to disable
    disk_volume_paths: [ '/', '/dev/shm' ] # volumes to monitor, must be accessible from the container, [ '/' ] default

    wifi_interface: 'wlan0' # wi-fi interface to monitor, disabled if ''
    wifi_monitor_topic: '/iw_status' # writes output here
    enable_wifi_scan: True # enable wi-fi scanning, must be also enabled in Bridge UI config
    enable_wifi_roam: False # enable wi-fi roaming, must be also enabled in Bridge UI config
```

### Add the Agent service to your compose.yaml
Add phntm_agent service to your `~/compose.yaml` file with `~/phntm_agent.yaml` mounted in the container as shown below.
See available pre-built Docker images [here](https://ghcr.io/phantomcybernetics/phntm_bridge_agent).
```yaml
services:
  phntm_agent:

    # select a pre-built image according to your ROS distro
    image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-humble
    # image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-iron
    # image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-jazzy
    # image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-kilted
    # image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-lyrical
    # image: ghcr.io/phantomcybernetics/phntm_bridge_agent:main-rolling
    # or image: phntm/agent:humble if built from source (see below)

    container_name: phntm-agent
    hostname: phntm-agent.local
    restart: unless-stopped # restarts after first run
    privileged: true # agent needs this
    network_mode: host # webrtc needs this
    ipc: host # agent needs this to see other local containers
    # environment:
    #  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # recommended, see Bridge Client instructions
    #  - ROS_DOMAIN_ID=22 # if used, specify ROS domain ID here
    volumes:
      # - ~/phntm_agent:/ros2_ws/src/phntm_agent # (optional) live repo mapped here for easy updates
      - ~/phntm_agent.yaml:/ros2_ws/phntm_agent_params.yaml # agent config goes here
      - /var/run:/host_run # docker file extractor and wi-fi control need this
      - /tmp:/tmp # wi-fi control needs this
    command:
      ros2 launch phntm_agent agent_launch.py
```

### Launch
```bash
docker compose up phntm_agent
```

## (Optional) Clone this repo and build the Docker image from source
```bash
cd ~
git clone git@github.com:PhantomCybernetics/phntm_agent.git phntm_agent
cd phntm_agent
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/agent:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .
# then use "image: phntm/agent:$ROS_DISTRO" in your ~/compose.yaml
```

## Upgrading
You may want to check out and/or follow our [Bluesky account](https://bsky.app/profile/phntm.io) for updates and service announcements. Significant milestones and interesting new features will be also e-mailed to the maintainer's e-mail address.

```bash
# Stop and remove the current Docker Container
docker stop phntm-agent && docker rm phntm-agent

# If using pre-built Docker Images, run:
docker image rm ghcr.io/phantomcybernetics/phntm_bridge_agent:main-humble
docker compose pull phntm_agent

# If building from source:
docker image rm phntm/agent:humble
cd ~/phntm_agent
git pull
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/agent:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .

# All done, relaunch
docker compose up phntm_agent
```

## See also
- [Documentation](https://docs.phntm.io/bridge) Full Phantom Bridge documentation
- [Phantom Brige Client](https://github.com/PhantomCybernetics/phntm_bridge_client) Phantom Bridge Client repo and install instructions
