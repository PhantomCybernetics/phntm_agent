import rclpy
from rclpy.node import Node, Parameter, QoSProfile, Publisher
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration, Infinite
from rclpy.serialization import deserialize_message

from .inc.status_led import StatusLED
from termcolor import colored as c

import docker
docker_client = None
try:
    host_docker_socket = 'unix:///host_run/docker.sock' # link /var/run/ to /host_run/ in docker-compose
    # host_docker_socket = 'tcp://0.0.0.0:2375'
    docker_client = docker.DockerClient(base_url=host_docker_socket)
except Exception as e:
    print(c(f'Failed to init docker client with {host_docker_socket} {e}', 'red'))
    pass

def main(): # ros2 calls this, so init here
    # asyncio.set_event_loop_policy(MyPolicy())
    # try:
    #     asyncio.run(main_async())
    # except (asyncio.CancelledError, KeyboardInterrupt):
    #     pass
    print('Hello from Agent')

if __name__ == '__main__': #ignired by ros
    main()