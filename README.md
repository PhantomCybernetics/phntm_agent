# Phantom Bridge Agent

This is a supporting package for the Phantom Brige. It monitors system resources (such as memory, disk usage and CPU load), monitors Wi-Fi connection quality (allowing to scan and roam between APs), allows to control configured Docker containers and to monitor their resource use. It also enables extraction of files from all running Docker containers.

Typically, this package is installed with the Phantom Bridge and runs inside its Docker container. It can be also installed in a standalone mode, which is useful for distributed ROS2 systems. 

## Install (Standalone)

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
Here's an example config file, e.g. `~/phntm_agent.yaml`.
```yaml
/**:
  ros__parameters:
    host_name: 'pi5' # lower case, must be valid ros id or ''
    refresh_period_sec: 0.5
    docker: True # monitor containers
    docker_topic: '/docker_info'
    docker_control: True
    system_info: True # monitor system stats
    system_info_topic: '/system_info_pi5'
    disk_volume_paths: [ '/', '/dev/shm' ] # volumes to monitor, must be accessible from the container
    iw_interface: 'wlan0' # disabled if empty
    iw_monitor_topic: '/iw_status' # writes output here
    iw_control: True # enable wi-fi scanning
    iw_roaming: False # enable wi-fi roaming
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
    # cpuset: '0,1,2' # consider dedicating a few CPU cores for maximal responsiveness
    network_mode: host # webrtc needs this
    ipc: host # agent needs this to see other local containers
    volumes:
      - ~/phntm_agent:/ros2_ws/src/phntm_agent # live repo mapped here for easy updates
      - ~/phntm_agent.yaml:/ros2_ws/phntm_agent_params.yaml # agent config goes here
      - /var/run:/host_run # docker file extractor and wifi control need this
      - /tmp:/tmp # wifi control needs this
    devices:
      - /dev:/dev # LED control needs this
    command:
      ros2 launch phntm_agent agent_launch.py
```

### Launch
```bash
docker compose up phntm_agent
```

## Upgrading

Unless the Dockerfile changes between versions (which doesn't happen very often), all you need to do to upgrade the Phantom Agent is to pull updates from this repo and restart the Docker container.

```bash
cd ~/phntm_agent
git pull
docker restart phntm-agent
```

Should the Dockerfile change, you need to rebuild the Docker image too.

## See also
- [Documentation](https://docs.phntm.io/bridge) Full Phantom Bridge documentation