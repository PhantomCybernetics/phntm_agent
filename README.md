# Phantom Bridge Agent

This is a supporting package for the [Phantom Brige Client](https://github.com/PhantomCybernetics/phntm_bridge_client).

The Agent monitors system resources (such as memory, disk usage and CPU load), Wi-Fi connection quality (allowing to scan and roam between APs), allows to control configured Docker containers and to monitor their used resources. It also enables extraction of files from any running Docker container, identified either with absolute path or ROS2 package:// format.

Typically, the Agent is installed with the Phantom Bridge Client and runs inside its Docker container, in which case it can also share one config file with the Client.
However, it can be also installed in a standalone mode, which is useful for monitoring and control of distributed ROS2 systems. 

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

### Clone this repo and build the Docker image
```bash
cd ~
git clone git@github.com:PhantomCybernetics/phntm_agent.git phntm_agent
cd phntm_agent
ROS_DISTRO=humble; docker build -f Dockerfile -t phntm/agent:$ROS_DISTRO --build-arg ROS_DISTRO=$ROS_DISTRO .
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
Add phntm_agent service to your `~/compose.yaml` file with `~/phntm_agent.yaml` mounted in the container as shown below:
```yaml
services:
  phntm_agent:
    image: phntm/agent:humble
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

## Upgrading

Unless the Dockerfile changes between versions (which doesn't happen very often), all you need to do to upgrade the Phantom Agent is to pull updates from this repo and then restart the Docker container.

```bash
cd ~/phntm_agent
git pull
docker restart phntm-agent
```

Should the Dockerfile change, you need to rebuild the Docker image too.

## See also
- [Documentation](https://docs.phntm.io/bridge) Full Phantom Bridge documentation
- [Phantom Brige Client](https://github.com/PhantomCybernetics/phntm_bridge_client) Phantom Bridge Client repo and install instructions
