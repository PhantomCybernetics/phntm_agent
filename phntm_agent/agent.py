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
from phntm_interfaces.msg import DockerStatus, DockerContainerStatus, CPUStatusInfo, DiskVolumeStatusInfo, SystemInfo, IWStatus, IWScanResult
from phntm_interfaces.srv import DockerCmd, IWScanCmd
from .lib import format_bytes, clear_line, print_line, set_message_header

import docker
docker_client = None
try:
    host_docker_socket = 'unix:///host_run/docker.sock' # link /var/run/ to /host_run/ in docker-compose
    # host_docker_socket = 'tcp://0.0.0.0:2375'
    docker_client = docker.DockerClient(base_url=host_docker_socket)
except Exception as e:
    print(c(f'Failed to init docker client with {host_docker_socket} {e}', 'red'))
    pass

import iwlib
import iwlib.iwlist

class AgentController(Node):
    ##
    # node constructor
    ##
    def __init__(self):
        
        self.shutting_down:bool = False
        
        node_name ='phntm_agent'
        self.hostname = ''
        
        # load node name from config before we can set node name
        config_path = os.path.join(
            '/ros2_ws/',
            'phntm_agent_params.yaml'
            )
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.hostname = config["/**"]["ros__parameters"].get('host_name', 'localhost')
                node_name = f'{node_name}_{self.hostname}'
        except FileNotFoundError:
            pass
        
        super().__init__(node_name=f'{node_name}',
                         use_global_arguments=True)
        
        self.load_config()  # load the rest the ros way
       
        self.l = self.get_logger()
        self.l.set_level(rclpy.logging.LoggingSeverity.DEBUG) 
        self.l.debug(f'Phntm Agent{" @ "+self.hostname if self.hostname != "" else ""} started')    
        
        self.last_printed_lines = 0
        self.docker_pub = None
        self.docker_sub = None
        self.sysinfo_pub = None
        self.iw_pub = None
        
        if self.iw_enabled:
            self.iw_max_quality:float = iwlib.utils.get_max_quality(self.iw_interface)
            self.iw_supports_scanning:bool = iwlib.utils.supports_scanning(self.iw_interface)
            self.last_essid:str = None #roaming between APs with same essid
            self.last_access_point:str = None
            self.last_frequency:float = None #GHz
        
        self.docker_cmd_srv = self.create_service(DockerCmd, f'/{node_name}/docker_command', self.docker_command_srv_callback)
        self.iw_scan_cmd_srv = self.create_service(IWScanCmd, f'/{node_name}/iw_scan', self.iw_scan_command_srv_callback)


    def docker_command_srv_callback(self, request, response):
        response.err = 0
        
        if not self.docker_control_enabled:
            response.err = 3
            response.msg = 'Docker control disabled'
            return response
        
        self.get_logger().debug(f'Docker request {request.id_container} state:{request.set_state}')
        
        try:
            cont = docker_client.containers.get(request.id_container)
        except docker.errors.APIError:
            response.err = 3
            response.msg = 'Docker container not found for id='+request.id_container
            return response
        
        try:
            match request.set_state:
                case 1:
                    if cont.status == 'running':
                        response.err = 3
                        response.msg = 'Container already running'
                        return response
                    cont.start()
                case 0:
                    if cont.status == 'exited':
                        response.err = 3
                        response.msg = 'Container already exited'
                        return response
                    cont.stop(timeout=3)
                case 2:
                    if cont.status == 'restarting':
                        response.err = 3
                        response.msg = 'Container already restarting'
                        return response
                    cont.restart(timeout=3)
        except Exception as e:
            response.err = 3
            response.msg = f'Docker exception: {str(e)}'
            return response
        
        return response


    def iw_scan_command_srv_callback(self, request, response):
        response.err = 0
        
        self.get_logger().info(c(f'IW scan request received; roam={request.attempt_roam}', 'cyan'))
                
        if not self.iw_supports_scanning:
            response.err = 3
            response.msg = 'Interface doesn\'t support scanning'
            return response

        if not self.iw_control_enabled:
            response.err = 3
            response.msg = 'Wifi control disabled by Agent'
            return response
        
        try:
            results = iwlib.iwlist.scan(self.iw_interface)
        except Exception as e:
             print(f'Exception while scanning IW: {e}')
             response.err = 3
             response.msg = f'Exception while scanning: {str(e)}'
             return response

        print(f'IW Monitor scan results: ')
        
        response.scan_results = []
        roaming_candidates = []

        for one_res in results:
            one_data = IWScanResult()
            linehr = []
            print(f'{one_res}')
            if 'ESSID' in one_res.keys():
                one_data.essid = one_res['ESSID'].decode() # b'CircuitLaunch'
                linehr.append(f'ESSID: {one_data.essid}')

            if 'Access Point' in one_res.keys():
                one_data.access_point = one_res['Access Point'].decode() # b'BA:FB:E4:45:19:4F'
                linehr.append(f'AP: {one_data.access_point}')

            if 'Frequency' in one_res.keys():
                one_data.frequency = float(one_res['Frequency'].split()[0]) # b'5.24 GHz'
                linehr.append(f'Freq: {one_data.frequency} GHz')

            if 'BitRate' in one_res.keys():
                one_data.bit_rate = float(one_res['BitRate'].split()[0]) # b'120 Mb/s'
                linehr.append(f'BR: {one_data.bit_rate} Mb/s')

            if 'Mode' in one_res.keys():
                if one_res['Mode'] == b'Managed':
                    one_data.mode = IWScanResult.MODE_MANAGED #b'Managed'
                elif one_res['Mode'] == b'Ad-Hoc':
                    one_data.mode = IWScanResult.MODE_AD_HOC #b'Ad-Hoc'
                elif one_res['Mode'] == b'Master':
                    one_data.mode = 3 # something else
                linehr.append(f'Mode: {one_data.mode}')

            if 'stats' in one_res.keys():
                if 'quality' in one_res['stats'].keys():
                    one_data.quality = one_res['stats']['quality'] # 34
                    linehr.append(f'Q: {one_data.quality}')

                if 'level' in one_res['stats'].keys():
                    one_data.level = one_res['stats']['level'] # 180
                    linehr.append(f'Lvl: {one_data.level}')

                if 'noise' in one_res['stats'].keys():
                    one_data.noise = one_res['stats']['noise'] # 0
                    linehr.append(f'N: {one_data.noise}')

                if 'updated' in one_res['stats'].keys():
                    one_data.updated = one_res['stats']['updated']
                    linehr.append(f'Upd: {one_data.updated}')

            if one_data.essid == self.last_essid \
            and one_data.access_point and one_data.quality and one_data.level:
                one_data.roaming_candidate = True
                roaming_candidates.append(one_data)

            response.scan_results.append(one_data)
            print(c(', '.join(linehr), 'cyan'))

        if request.attempt_roam:
            if not self.iw_roaming_enabled:
                response.err = 3
                response.msg = 'Roaming disabled by Agent'
            else:
                print(f'IW: Seeing {len(roaming_candidates)} roaming candidates for {self.last_essid}')
                # sort by quallity desc
                sorted_candidates = sorted(roaming_candidates, key=lambda x: x.level, reverse=True)
                for cand in sorted_candidates:
                    print(f" >> {cand.access_point if cand.access_point else 'N/A'} {cand.frequency} GHz, Quality={cand.quality}, lvl={cand.level}, noise={cand.noise} {' < CURR' if cand.access_point == self.last_access_point else ''}")

                bestest = sorted_candidates[0]
                if bestest.access_point == self.last_access_point:
                    response.res = 0
                    response.msg = 'Not roaming, current network seems best'
                    print(c(f" >>> Not roaming, current AP seems best", 'cyan'))
                else:
                    print(c(f" >>> Attenmpting to roam to {bestest.access_point} with quality={bestest.quality}", 'cyan'))
                    wpa_cli_res = os.system(f"wpa_cli -p /host_run/wpa_supplicant/ -i {self.iw_interface} roam {bestest.access_point}")
                    print(f'wpa_cli_res={wpa_cli_res}')
                    response.res = wpa_cli_res
                    response.msg = bestest.access_point

        return response
    
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

    
    async def get_docker_containers(self):
        docker_containers = docker_client.containers.list(all=True)
         
        msg = DockerStatus()
        set_message_header(self, msg)
        msg.containers = []
         
        c_stats = []
        for cont in docker_containers:
            msg_cont = DockerContainerStatus()
            cs = {}
            if cont.status == 'running':
                if not cont.id in self.docker_stats_streams:
                    self.docker_stats_streams[cont.id] = cont.stats(stream=True, decode=True)
                stats = next(self.docker_stats_streams[cont.id])
                
                # stats_decoded = json.loads(stats)
                # print(json.dumps(stats, indent=4))
                cs = self.calculate_docker_stats(stats)
                msg_cont.pids = cs['pids']
                msg_cont.cpu_percent = cs['cpu_perc']
                msg_cont.block_io_read_bytes = cs['block_io_read']
                msg_cont.block_io_write_bytes = cs['block_io_write']
            else:
                msg_cont.pids = 0
                msg_cont.cpu_percent = 0.0
                msg_cont.block_io_read_bytes = 0
                msg_cont.block_io_write_bytes = 0
                
            cs['name'] = cont.name
            cs['status'] = cont.status
            cs['short_id'] = cont.short_id
            cs['id'] = cont.id
            msg_cont.name = cont.name
            msg_cont.id = cont.id
            msg_cont.status = DockerContainerStatus.STATUS_EXITED
            match cont.status:
                case 'restarting':
                    msg_cont.status = DockerContainerStatus.STATUS_RESTARTING
                case 'running':
                    msg_cont.status = DockerContainerStatus.STATUS_RUNNING
                case 'paused':
                    msg_cont.status = DockerContainerStatus.STATUS_PAUSED
                case 'exited':
                    msg_cont.status = DockerContainerStatus.STATUS_EXITED
            c_stats.append(cs)
            msg.containers.append(msg_cont)

        for i in range(len(c_stats)):
            cs = c_stats[i]
            clr = 'red'
            match cs['status']:
                case 'running': clr = 'green'
                case 'exited': clr = 'red'
                case _: clr = 'cyan'
            
            if cs['status'] == 'running':
                print_line(self, f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}] CPU: {cs["cpu_perc"]:.2f}% BLOCK I/O: {format_bytes(cs["block_io_read"], True)} / {format_bytes(cs["block_io_write"], True)} PIDS: {str(cs["pids"])}')
            else:
                print_line(self, f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}]')
        
        if self.docker_pub and self.context.ok():
            self.docker_pub.publish(msg)
    
    
    async def get_system_info(self):
        cpu_count = psutil.cpu_count()
        cpu_times = psutil.cpu_times_percent(interval=1, percpu=True)
        mem = psutil.virtual_memory()
        swp = psutil.swap_memory()
        
        msg = SystemInfo()
        set_message_header(self, msg)
        
        msg.cpu = []
        i = 0
        for cpu in cpu_times:
            print_line(self, f'[CPU {str(i)}] User:{cpu.user:.1f}% Nice:{cpu.nice:.1f}% Sys:{cpu.system:.1f}% Idle:{cpu.idle:.1f}% ... {100.0-cpu.idle:.1f}%')
            i += 1
            msg_cpu = CPUStatusInfo()
            msg_cpu.user_percent = cpu.user
            msg_cpu.nice_percent = cpu.nice
            msg_cpu.system_percent = cpu.system
            msg_cpu.idle_percent = cpu.idle
            msg.cpu.append(msg_cpu)
        
        print_line(self, f'[MEM] Tot:{format_bytes(mem.total)} Avail:{format_bytes(mem.available)} Used:{format_bytes(mem.used)} Free:{format_bytes(mem.free)} Buff:{format_bytes(mem.buffers)} Shar:{format_bytes(mem.shared)} Cach:{format_bytes(mem.cached)}')
        msg.mem_total_bytes = mem.total
        msg.mem_available_bytes = mem.available
        msg.mem_used_bytes = mem.used
        msg.mem_free_bytes = mem.free
        msg.mem_buffers_bytes = mem.buffers
        msg.mem_shared_bytes = mem.shared
        msg.mem_cached_bytes = mem.cached

        print_line(self, f'[SWP] Tot:{format_bytes(swp.total)} Used:{format_bytes(swp.used)} Free:{format_bytes(swp.free)}')
        msg.swp_total_bytes = swp.total
        msg.swp_used_bytes = swp.used
        msg.swp_free_bytes = swp.free
        
        i = 0
        msg.disk = []
        for disk_path in self.disk_paths:
            dsk = psutil.disk_usage(disk_path)
            i += 1
            print_line(self, f'[DSK {disk_path}] Tot:{format_bytes(dsk.total)} Used:{format_bytes(dsk.used)} Free:{format_bytes(dsk.free)}')    
            msg_dsk = DiskVolumeStatusInfo()
            msg_dsk.path = disk_path
            msg_dsk.total_bytes = dsk.total
            msg_dsk.used_bytes = dsk.used
            msg_dsk.free_bytes = dsk.free
            msg.disk.append(msg_dsk)
        
        if self.sysinfo_pub and self.context.ok():
            self.sysinfo_pub.publish(msg)


    async def get_iw_info(self):
        
        cfg = await asyncio.get_event_loop().run_in_executor(None, iwlib.iwconfig.get_iwconfig, self.iw_interface)
        msg = IWStatus()

        set_message_header(self, msg)

        try:
            if 'Frequency' in cfg:
                msg.frequency = float(cfg['Frequency'].split()[0]) # b'5.24 GHz'
            if 'Access Point' in cfg:
                msg.access_point = cfg['Access Point'].decode() # b'BA:FB:E4:45:19:4F'
            if 'BitRate' in cfg:
                msg.bit_rate = float(cfg['BitRate'].split()[0]) # b'120 Mb/s'
            if 'ESSID' in cfg:
                msg.essid = cfg['ESSID'].decode() # b'CircuitLaunch'
            if 'Mode' in cfg:
                if cfg['Mode'] == b'Managed':
                    msg.mode = IWStatus.MODE_MANAGED #b'Managed'
                elif cfg['Mode'] == b'Ad-Hoc':
                    msg.mode = IWStatus.MODE_AD_HOC #b'Ad-Hoc'
            if 'stats' in cfg:
                if 'quality' in cfg['stats']:
                    msg.quality = cfg['stats']['quality'] # 34
                if 'level' in cfg['stats']:
                    msg.level = cfg['stats']['level'] # 180
                if 'noise' in cfg['stats']:
                    msg.noise = cfg['stats']['noise'] # 0

            msg.quality_max = self.iw_max_quality # 70
            msg.supports_scanning = self.iw_supports_scanning
            
            # msg.num_peers = len(self.wrtc_peers)
            
            self.last_essid = msg.essid
            self.last_access_point = msg.access_point
            self.last_frequency = msg.frequency

            print_line(self, f'[NET] Q:{str(msg.quality)}% L:{str(msg.level)} N:{str(msg.noise)} AP:{msg.access_point}')
        
            if self.iw_pub and self.context.ok():
                self.iw_pub.publish(msg)

        except Exception as e:
            print (c(f'Error while generating IWStatus: {e}', 'red'))
            print (f'IW CFG was: {cfg}')
            
    
    async def agent_loop(self):

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
        
        if self.system_info_enabled:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                             depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT
                             )
            self.sysinfo_pub = self.create_publisher(SystemInfo, self.system_info_topic, qos)
            if self.sysinfo_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.system_info_topic}, msg_type=SystemInfo')
                self.system_info_enabled = False
                
        if self.iw_interface and self.iw_monitor_topic:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                             depth=1,
                             reliability=QoSReliabilityPolicy.BEST_EFFORT
                             )
            self.iw_pub = self.create_publisher(IWStatus, self.iw_monitor_topic, qos)
            if self.iw_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.iw_monitor_topic}, msg_type=IWStatus')
                self.iw_enabled = False
        
        while not self.shutting_down:
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if not self.scroll_enabled and self.last_printed_lines > 0:
                for i in range(self.last_printed_lines+1):
                    sys.stdout.write("\033[F") # Cursor up one line
                sys.stdout.write("\r\n")
                clear_line(self)
                self.last_printed_lines = 0
                
            print_line(self, f'Introspecting ({str(c)})...')
            c += 1 # counter

            if self.docker_enabled:
                await self.get_docker_containers()
            if self.system_info_enabled:
                await self.get_system_info()
            if self.iw_enabled:
                await self.get_iw_info()
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
        self.declare_parameter('docker_topic', '/docker_info')
        self.docker_topic = self.get_parameter('docker_topic').get_parameter_value().string_value
        if self.docker_enabled:
            print(f'Monitoring Docker -> {self.docker_topic}')
            
        self.declare_parameter('docker_control', True)
        self.docker_control_enabled = self.get_parameter('docker_control').get_parameter_value().bool_value
        if self.docker_control_enabled:
            print(f'Docker control enabled')
        
        self.declare_parameter('system_info', True)
        self.system_info_enabled = self.get_parameter('system_info').get_parameter_value().bool_value
        self.declare_parameter('system_info_topic', '/system_info')
        self.system_info_topic = self.get_parameter('system_info_topic').get_parameter_value().string_value
        if self.system_info_enabled:
            print(f'System monitoring CPU/MEM/SWP+disks -> {self.system_info_topic}')
      
        self.declare_parameter('disk_volume_paths', [ '/' ]) 
        self.disk_paths = self.get_parameter('disk_volume_paths').get_parameter_value().string_array_value
        if self.system_info_enabled:
            print(f'Monitoring disk volumes: {str(self.disk_paths)}')
            
        self.declare_parameter('iw_interface', 'wlan0')
        self.iw_interface = self.get_parameter('iw_interface').get_parameter_value().string_value
        self.declare_parameter('iw_monitor_topic', '/iw_status')
        self.iw_monitor_topic = self.get_parameter('iw_monitor_topic').get_parameter_value().string_value
        self.iw_enabled = self.iw_interface and self.iw_monitor_topic
        if self.iw_enabled:
            print(f'Monitoring netwrork interface {self.iw_interface} -> {self.iw_monitor_topic}')

        self.declare_parameter('iw_control', True)
        self.iw_control_enabled = self.get_parameter('iw_control').get_parameter_value().bool_value
        self.declare_parameter('iw_roaming', True)
        self.iw_roaming_enabled = self.get_parameter('iw_roaming').get_parameter_value().bool_value
        if self.iw_control_enabled:
            print(f'Network control enabled'+(' with roaming' if self.iw_roaming_enabled else ''))
    
    async def shutdown_cleanup(self):
        if self.docker_pub:
            self.docker_pub.destroy()
            self.docker_pub = None
            
        if self.sysinfo_pub:
            self.sysinfo_pub.destroy()
            self.sysinfo_pub = None
            
        if self.iw_pub:
            self.iw_pub.destroy()
            self.iw_pub = None


async def main_async(args):
    rclpy.init()
    
    try:
        agent_node = AgentController()
        loop_task = asyncio.get_event_loop().create_task(agent_node.agent_loop(), name="introspection_task")
        await asyncio.wait([ loop_task ], return_when=asyncio.ALL_COMPLETED)
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