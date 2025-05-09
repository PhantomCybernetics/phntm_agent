import asyncio

from rclpy.node import Node, Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from phntm_interfaces.msg import FileChunk

import subprocess
import os
import time
import math
import docker
import tarfile, io

def locate_file(file_url:str, ros_distro:str, docker_client:docker.DockerClient, logger:RcutilsLogger):
    
    pkg:str = None
    local_pkg_prefix = ""
    
    if file_url.startswith('file:/'):
        file_url = file_url.replace('file://', '')
        file_url = file_url.replace('file:/', '')
        if not file_url.startswith('/'):
            file_url = '/' + file_url
        logger.info(f'Bridge requesting file {file_url}')
        
    elif file_url.startswith('package:/'):
        file_url = file_url.replace('package://', '')
        file_url = file_url.replace('package:/', '')
        
        parts = file_url.split('/')
        pkg = parts[0]
        
        if not pkg:
            logger.error(f'Bridge requested invalid file, no package name provided in {file_url}')
            return None # file not found
        
        if not file_url.startswith('/'):
            file_url = '/' + file_url
        
        logger.info(f"Bridge requesting file {file_url} in pkg '{pkg}'")
        
        if pkg is not None:
            res = subprocess.run([f"/opt/ros/{ros_distro}/bin/ros2", "pkg", "prefix", pkg], capture_output=True)
            if res.stdout:
                local_pkg_prefix = res.stdout.decode("ASCII").rstrip() + '/share'
                logger.debug(f"Local pkg prefix is {local_pkg_prefix}")
            else:
                logger.debug(f"Local prefix for pkg '{pkg}' not found in this fs")
    else:
        logger.error(f'Bridge requested invalid file {file_url}')
        return None # file not found
    
    # package file in local fs
    if local_pkg_prefix and os.path.isfile(local_pkg_prefix + file_url):
        logger.debug(f'File found in this fs (pkg_prefix={local_pkg_prefix})')
        f = open(file_url, "rb")  # follows symlinks
        res = f.read()
        f.close()
        return res
    
    # absolute path in local fs
    elif os.path.isfile(file_url):
        logger.debug(f'File found in this fs')
        f = open(file_url, "rb") # follows symlinks
        res = f.read()
        f.close()
        return res
    
    # inspect other containers
    elif docker_client:
        logger.debug(f'File not found in this fs, searching other Docker containers...')
        docker_containers = docker_client.containers.list(all=False)
        for container in docker_containers:
            cont_pkg_prefix = ""    
            if pkg:
                # cmd = f'/bin/bash -c "export PS1=phntm && . /opt/ros/{ros_distro}/setup.bash && . ~/.bashrc && /opt/ros/{ros_distro}/bin/ros2 pkg prefix {pkg}"'
                cmd = (
                    '/bin/bash -c "'
                    'export PS1=phntm && '
                    '. /opt/ros/$ROS_DISTRO/setup.bash && '
                    '. ~/.bashrc && '
                    'if [ -n \\"$ROS_WS\\" ]; then '
                    '  . $ROS_WS/install/setup.bash; '
                    'fi && '
                    f'/opt/ros/$ROS_DISTRO/bin/ros2 pkg prefix {pkg}'
                    '"'
                )
                res = container.exec_run(cmd)
                if res.exit_code == 1:
                    logger.debug(f'Pkg not found in container {container.name} \nout={res.output}\ncmd={cmd}')
                    continue
                else:
                    cont_pkg_prefix = res.output.decode("ASCII").rstrip() + '/share'
                    logger.debug(f'Container {container.name} has pkg in {cont_pkg_prefix}')
            
            tar_chunks = None
            try:
                tar_chunks, stats = container.get_archive(cont_pkg_prefix+file_url, chunk_size=None, encode_stream=False)
                while stats['linkTarget']:
                    logger.debug(f"Following symlink to {stats['linkTarget']} fs")
                    tar_chunks, stats = container.get_archive(stats['linkTarget'], chunk_size=None, encode_stream=False)
            except Exception as e:
                logger.debug(f'File not found in {container.name} fs')
                continue
            
            logger.debug(f'File found in {container.name} fs')
            logger.debug(str(stats))
            
            b_arr = []
            for chunk in tar_chunks:
                b_arr.append(chunk)
            
            tar_bytes = b''.join(b_arr)
            
            logger.debug(f'Making tar obj w {len(tar_bytes)} B')
            
            file_like_object = io.BytesIO(tar_bytes)
            tar = tarfile.open(fileobj=file_like_object)

            member = tar.getmember(stats['name'])
                        
            res_bytes = tar_bytes[member.offset_data : member.offset_data+stats['size']]
            return res_bytes
            
    return None # file not found


async def produce_file_chunks(file_path:str, file_bytes:bytes, byte_size:int, chunk_size:int, num_parts:int, pub:Publisher, node:Node, logger:RcutilsLogger):
    
    await asyncio.sleep(0.01) # wait a bit to make sure the sending starts after the service reply
    
    logger.info(f' Producing {byte_size}B as {num_parts} chunks')
    
    offset = 0
    for index in range(num_parts):
        
        msg = FileChunk()
        msg.file_path = file_path
        msg.chunk_number = index
        msg.total_chunks = num_parts
        msg.data = file_bytes[offset : offset+chunk_size]
    
        offset += chunk_size
        
        if node.context.ok():
            pub.publish(msg)
            logger.debug(f"Produced chunk {index + 1}/{num_parts}")
        else:
            logger.error(f"Failed producing chunk {index + 1}/{num_parts}")
            
            
def set_message_header(node, msg):
    time_nanosec:int = time.time_ns()
    msg.header.stamp.sec = math.floor(time_nanosec / 1000000000)
    msg.header.stamp.nanosec = time_nanosec % 1000000000
    msg.header.frame_id = node.hostname


def format_bytes(b, mib=False):        
    unit = 1000
    GB = unit * unit * unit # 
    MB = unit * unit # docker stats shows MiB, keep consistent
    KB = unit
    
    if b > GB:
        return f'{(b / GB):.2f}{"GiB" if mib else "GB"}'
    elif b > MB:
        return f'{(b / MB):.2f}{"MiB" if mib else "MB"}'
    elif b > KB:
        return f'{(b / KB):.2f}{"KiB" if mib else "KB"}'
    elif b > 0:
        return f'{(b):.2f}B'
    else:
        return f'0B'