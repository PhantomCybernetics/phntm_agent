import rclpy
from rclpy.node import Node, Parameter, QoSProfile, Publisher
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration, Infinite
from rclpy.serialization import deserialize_message
import asyncio
import traceback
import selectors
import os
from termcolor import colored as c
import yaml
import json
import sys
import psutil
import math
import time
from phntm_interfaces.msg import DockerStatus

import docker
docker_client = None
try:
    host_docker_socket = 'unix:///host_run/docker.sock' # link /var/run/ to /host_run/ in docker-compose
    # host_docker_socket = 'tcp://0.0.0.0:2375'
    docker_client = docker.DockerClient(base_url=host_docker_socket)
except Exception as e:
    print(c(f'Failed to init docker client with {host_docker_socket} {e}', 'red'))
    pass



class AgentController(Node):
    ##
    # node constructor
    ##
    def __init__(self):
        
        self.shutting_down:bool = False
        
        # load node name from config
        config_path = os.path.join(
            '/ros2_ws/',
            'phntm_agent_params.yaml'
            )
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            self.hostname = config["/**"]["ros__parameters"].get('host_name', 'localhost')
        
        super().__init__(node_name=f'phntm_agent_{self.hostname}',
                         use_global_arguments=True)
        
        self.load_config()  # load the rest the ros way
       
        self.l = self.get_logger()
        self.l.set_level(rclpy.logging.LoggingSeverity.DEBUG) 
        self.l.debug(f'Phntm Agent @ {self.hostname} started')    
        
        self.last_printed_lines = 0
        self.docker_pub = None
        self.docker_sub = None
        self.top_pub = None
        self.disk_pub = None
    
    
    def calculate_docker_stats(self, stats):

        # mem_bytes_used = stats["memory_stats"]["usage"]
        # mem_bytes_avail = stats["memory_stats"]["limit"]
        # mem_gb_used = round(mem_bytes_used / (1024*1024*1024), 1) 
        # mem_gb_avail = round(mem_bytes_avail / (1024*1024*1024), 1) 
        res = {
            'num_cpus': 0,
            'cpu_perc': 0.0,
            'cpu_max_perc': 0.0,
            'pids': 0,
            'block_io_read': 0,
            'block_io_write': 0,
            'mem': 0, #TODO ignoring for now, mem not working on linux
            'mem_perc': 0.0,
            'net_read': 0, #TODO ignoring for now, net not working on linux
            'net_write': 0,
        }
        
        if 'pids_stats' in stats and 'current' in stats['pids_stats']:
            res['pids'] = stats['pids_stats']['current']
        
        if 'cpu_stats' in stats and 'system_cpu_usage' in stats['cpu_stats'] \
        and 'precpu_stats' in stats and 'system_cpu_usage' in stats['precpu_stats']:
            cpu_usage = (stats['cpu_stats']['cpu_usage']['total_usage']
                        - stats['precpu_stats']['cpu_usage']['total_usage'])
            cpu_system = (stats['cpu_stats']['system_cpu_usage']                    
                        - stats['precpu_stats']['system_cpu_usage'])
            res['num_cpus'] = stats['cpu_stats']["online_cpus"]
            res['cpu_perc'] = (cpu_usage / cpu_system) * res['num_cpus'] * 100.0
            res['cpu_max_perc'] = res['num_cpus'] * 100

        if 'blkio_stats' in stats and 'io_service_bytes_recursive' in stats['blkio_stats']\
        and stats['blkio_stats']['io_service_bytes_recursive'] != None:
            for blkio_stats in stats['blkio_stats']['io_service_bytes_recursive']:
                if blkio_stats['op'] == 'read':
                    res['block_io_read'] = blkio_stats['value']
                elif blkio_stats['op'] == 'write':
                    res['block_io_write'] = blkio_stats['value']

        return res
    
    
    def format_bytes(self, b, mib=False):
        
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
    
    def clear_line(self):
        if not self.scroll_enabled:
            sys.stdout.write("\033[K")
    
    async def get_docker_containers(self):
        docker_containers = docker_client.containers.list(all=True)
         
        msg = DockerStatus()
        time_nanosec:int = time.time_ns()
        msg.header.stamp.sec = math.floor(time_nanosec / 1000000000)
        msg.header.stamp.nanosec = time_nanosec % 1000000000
        msg.host_name = self.hostname
        msg.num_containers = len(docker_containers)
        msg.names = []
        msg.ids = []
        msg.states = []
        msg.cpu_percents = []
        msg.block_io_read_bytes = []
        msg.block_io_write_bytes = []
        msg.pids = []
         
        c_stats = []
        for cont in docker_containers:
            cs = {}
            cs['stats_keys'] = []
            
            if cont.status == 'running':
                if not cont.id in self.docker_stats_streams:
                    self.docker_stats_streams[cont.id] = cont.stats(stream=True, decode=True)
                stats = next(self.docker_stats_streams[cont.id])
                
                # stats_decoded = json.loads(stats)
                # print(json.dumps(stats, indent=4))
                cs = self.calculate_docker_stats(stats)
                cs['stats_keys'] = stats.keys()
                msg.pids.append(cs['pids'])
                msg.cpu_percents.append(cs['cpu_perc'])
                msg.block_io_read_bytes.append(cs['block_io_read'])
                msg.block_io_write_bytes.append(cs['block_io_write'])
            else:
                msg.pids.append(0)
                msg.cpu_percents.append(0.0)
                msg.block_io_read_bytes.append(0)
                msg.block_io_write_bytes.append(0)
                
            cs['name'] = cont.name
            cs['status'] = cont.status
            cs['short_id'] = cont.short_id
            cs['id'] = cont.id
            msg.names.append(cont.name)
            msg.ids.append(cont.id)
            i_status = DockerStatus.STATUS_EXITED
            match cont.status:
                case 'restarting':
                    i_status = DockerStatus.STATUS_RESTARTING
                case 'running':
                    i_status = DockerStatus.STATUS_RUNNING
                case 'paused':
                    i_status = DockerStatus.STATUS_PAUSED
                case 'exited':
                    i_status = DockerStatus.STATUS_EXITED
            msg.states.append(i_status)
            c_stats.append(cs)

        for i in range(len(c_stats)):
            cs = c_stats[i]
            clr = 'red'
            match cs['status']:
                case 'running': clr = 'green'
                case 'exited': clr = 'red'
                case _: clr = 'cyan'
            
            if cs['status'] == 'running':
                print(f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}] CPU: {cs["cpu_perc"]:.2f}% BLOCK I/O: {self.format_bytes(cs["block_io_read"], True)} / {self.format_bytes(cs["block_io_write"], True)} PIDS: {str(cs["pids"])}')
                self.clear_line()
                self.last_printed_lines += 1
                # print(f'Stats: {str(cs["stats_keys"])}', end=end)
                # self.last_printed_lines += 2
            else:
                print(f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}]')
                self.clear_line()
                self.last_printed_lines += 1
        
        if self.docker_pub and self.context.ok():
            self.docker_pub.publish(msg)
           
    async def get_top(self):
        cpu_count = psutil.cpu_count()
        cpu_times = psutil.cpu_times_percent(interval=1, percpu=True)
        mem = psutil.virtual_memory()
        swp = psutil.swap_memory()
        
        # print(f'[CPU] {cpu_count}', end='\n')
        # self.last_printed_lines += 1
        i = 0
        for cpu in cpu_times:
            print(f'[CPU {str(i)}] User:{cpu.user:.1f}% Nice:{cpu.nice:.1f}% Sys:{cpu.system:.1f}% Idle:{cpu.idle:.1f}% ... {100.0-cpu.idle:.1f}%')
            self.clear_line()
            i += 1
            self.last_printed_lines += 1
        # print(f'[MEM] {str(mem)}', end='\n')
        # print(f'[SWP] {str(swp)}', end='\n')
        
        print(f'[MEM] Tot:{self.format_bytes(mem.total)} Avail:{self.format_bytes(mem.available)} Used:{self.format_bytes(mem.used)} Free:{self.format_bytes(mem.free)} Buff:{self.format_bytes(mem.buffers)} Shar:{self.format_bytes(mem.shared)} Cach:{self.format_bytes(mem.cached)}')
        self.clear_line()
        # endl = '' if (not self.disk_enabled and make_last_line) else '\n'
        print(f'[SWP] Tot:{self.format_bytes(swp.total)} Used:{self.format_bytes(swp.used)} Free:{self.format_bytes(swp.free)}')
        self.clear_line()
        self.last_printed_lines += 2
    
    async def get_disk(self):
        i = 0
        for disk_path in self.disk_paths:
            dsk = psutil.disk_usage(disk_path)
            i += 1
            print(f'[DSK {disk_path}] Tot:{self.format_bytes(dsk.total)} Used:{self.format_bytes(dsk.used)} Free:{self.format_bytes(dsk.free)}')
            self.clear_line()
            self.last_printed_lines += 1
    
    async def introspection(self):

        c = 0 # pass counter        
        self.docker_stats_streams = {}
        
        if self.docker_enabled:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                             depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT
                             )
            self.docker_pub = self.create_publisher(DockerStatus, self.docker_topic, qos)
            if self.docker_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.docker_topic}, msg_type=DockerStatus')
                self.docker_enabled = False
        
        while not self.shutting_down:
            
            if not self.scroll_enabled and self.last_printed_lines > 0:
                for i in range(self.last_printed_lines+1):
                    sys.stdout.write("\033[F") # Cursor up one line
                sys.stdout.write("\r\n")
                
            print(f'Introspecting ({str(c)})...')
            self.clear_line()
            c += 1 # counter
            self.last_printed_lines = 1        

            if self.docker_enabled:
                await self.get_docker_containers()
            if self.top_enabled:
                await self.get_top()
            if self.disk_enabled:
                await self.get_disk()
            await asyncio.sleep(self.discovery_period)
            
        print(f'\nIntrospection stopped')
        
    
    def load_config(self):
        
        self.declare_parameter('scroll', False)
        self.scroll_enabled = self.get_parameter('scroll').get_parameter_value().bool_value
        
        self.declare_parameter('discovery_period_sec', 1.0)
        self.discovery_period = self.get_parameter('discovery_period_sec').get_parameter_value().double_value
        print(f'Discovery period is {self.discovery_period:.0f}s')
        
        self.declare_parameter('docker', True)
        self.docker_enabled = self.get_parameter('docker').get_parameter_value().bool_value
        self.declare_parameter('docker_topic', '/_phntm_docker')
        self.docker_topic = self.get_parameter('docker_topic').get_parameter_value().string_value
        if self.docker_enabled:
            print(f'Monitoring Docker -> {self.docker_topic}')
            
        self.declare_parameter('docker_control', True)
        self.docker_control_enabled = self.get_parameter('docker_control').get_parameter_value().bool_value
        self.declare_parameter('docker_control_topic', '/_phntm_docker_control')
        self.docker_control_topic = self.get_parameter('docker_control_topic').get_parameter_value().string_value
        if self.docker_control_enabled:
            print(f'Docker control enabled <- {self.docker_control_topic}')
        
        self.declare_parameter('top', True)
        self.top_enabled = self.get_parameter('top').get_parameter_value().bool_value
        self.declare_parameter('top_topic', '/_phntm_top')
        self.top_topic = self.get_parameter('top_topic').get_parameter_value().string_value
        if self.top_enabled:
            print(f'Monitoring CPU/MEM/SWP -> {self.top_topic}')
        self.declare_parameter('disk', True)
        self.disk_enabled = self.get_parameter('disk').get_parameter_value().bool_value
        self.declare_parameter('disk_paths', [ '/' ]) 
        self.disk_paths = self.get_parameter('disk_paths').get_parameter_value().string_array_value
        self.declare_parameter('disk_topic', '/_phntm_disk')
        self.disk_topic = self.get_parameter('disk_topic').get_parameter_value().string_value
        if len(self.disk_paths) == 0 or (len(self.disk_paths) == 1 and self.disk_paths[0] == ''):
            self.disk_enabled = False
        if self.disk_enabled:
            print(f'Monitoring disk space at {str(self.disk_paths)} -> {self.disk_topic}')
        
        
    async def shutdown_cleanup(self):
        if self.docker_pub:
            self.docker_pub.destroy()
            self.docker_pub = None


async def main_async(args):
    rclpy.init()
    
    try:
        
        agent_node = AgentController()
        introspection_task = asyncio.get_event_loop().create_task(agent_node.introspection(), name="introspection_task")
        await asyncio.wait([ introspection_task ], return_when=asyncio.ALL_COMPLETED)
        
    except Exception as e:
        print(c('Exception in main_async()', 'red'))
        traceback.print_exc(e)
    except (asyncio.CancelledError, KeyboardInterrupt):
        print(c('Shutting down main_async', 'red'))
    
    print('SHUTTING DOWN')
    agent_node.shutting_down = True
    await agent_node.shutdown_cleanup()
    try:
        agent_node.destroy_node()
        # rcl_executor.shutdown()
        rclpy.shutdown()
    except:
        pass


class MyPolicy(asyncio.DefaultEventLoopPolicy):
    def new_event_loop(self):
        selector = selectors.SelectSelector()
        return asyncio.SelectorEventLoop(selector)

def main(args=None): # ros2 calls this, so init here
    asyncio.set_event_loop_policy(MyPolicy())
    try:
        asyncio.run(main_async(args))
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass

if __name__ == '__main__': #ignired by ros
    main()