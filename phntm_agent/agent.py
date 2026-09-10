import rclpy
from rclpy.node import Node, Parameter, QoSProfile, Publisher
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration, Infinite
from rclpy.serialization import deserialize_message
import subprocess
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
import signal
from phntm_interfaces.msg import DockerStatus, DockerContainerStatus, CPUStatusInfo, DiskVolumeStatusInfo, SystemInfo, IWStatus, IWScanResult, FileExtractionRequest, FileExtractionResult, FileChunk
from phntm_interfaces.srv import DockerCmd, IWScanCmd
from .inc.lib import format_bytes, set_message_header, locate_file, produce_file_chunks, upload_file_chunk, upload_file_chunks, complete_file_upload
from std_msgs.msg import Int32

import docker
docker_client = None
try:
    host_docker_socket = 'unix:///host_run/docker.sock' # link /var/run/ to /host_run/ in docker-compose
    # host_docker_socket = 'tcp://0.0.0.0:2375'
    docker_client = docker.DockerClient(base_url=host_docker_socket)
except Exception as e:
    print(f'Failed to init docker client with {host_docker_socket} {e}')
    pass

import iwlib
import iwlib.iwlist

import sdbus
from sdbus_async.networkmanager import NetworkManager, NetworkDeviceGeneric
from sdbus_async.networkmanager.enums import DeviceType
import gi
gi.require_version("ModemManager", "1.0")
gi.require_version("Gio", "2.0")
from gi.repository import GLib, Gio, ModemManager
from enum import Flag

# from sdbus_async.modemmanager import MMModem #, Signal, Sim
# print(ModemManager.ModemAccessTechnology)
# for name in dir(ModemManager.ModemAccessTechnology):
#     if not name.startswith("_"):
#         print(name)
        
class AgentController(Node):

    ##
    # node constructor
    ##
    def __init__(self):
        
        self.shutting_down:bool = False
        
        self.node_name ='phntm_agent'
        self.hostname = ''
        
        # load node name from config before we can set node name
        config_path = os.path.join('/ros2_ws/', 'phntm_agent_params.yaml')
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.hostname = config["/**"]["ros__parameters"].get('host_name', 'localhost')
                self.node_name = f'{self.node_name}_{self.hostname}' if self.hostname else self.node_name
        except FileNotFoundError:
            pass
        
        super().__init__(node_name=f'{self.node_name}', use_global_arguments=True)
        
        self.load_config()  # load the rest the ros way
       
        self.l = self.get_logger()
        self.l.set_level(rclpy.logging.LoggingSeverity.DEBUG) 
        self.l.debug(f'Phntm Agent{" @ "+self.hostname if self.hostname != "" else ""} started')    
        
        if not self.log_output:
            self.l.info(f'Verbose logging disabled by config')
        
        if not docker_client:
            self.l.error(f'Docker client not available, did you mount \'{host_docker_socket}\'?')
        
        self.docker_pub = None
        self.docker_task = None
        self.sysinfo_pub = None
        self.sysinfo_task = None
        self.iw_device_type = IWStatus.DEVICE_TYPE_UNKNOWN
        self.iw_pub = None
        self.iw_task = None
        self.iw_modem_manager = None
        self.iw_modem_obj = None
        self.iw_modem = None
        self.file_request_sub = None
        self.file_result_pub = None
        self.file_chunk_pub = None
        self.file_chunk_sub = None
        
        self.iw_max_quality:float = False
        self.iw_supports_scanning:bool = False
        self.last_essid:str = None #roaming between APs with same essid
        self.last_access_point:str = None
        self.last_frequency:float = None #GHz
    
    
    async def setup(self):
        
        if self.iw_enabled:

            # DBUS_SYSTEM_BUS_ADDRESS=unix:path=/host_run/dbus/system_bus_socket
            sdbus.set_default_bus(sdbus.sd_bus_open_system())
            self.nm = NetworkManager()

            devices_paths = await self.nm.get_devices()
            for device_path in devices_paths:
                dev = NetworkDeviceGeneric(device_path)
                iface = await dev.interface
                dtype = DeviceType(await dev.device_type) 
            
                if iface == self.iw_interface:
                    if dtype == DeviceType.WIFI:
                        self.l.info(f'NM: {iface} type is WI-FI {device_path}')
                        self.iw_device_type = IWStatus.DEVICE_TYPE_WIFI
                        try:
                            self.iw_max_quality:float = iwlib.utils.get_max_quality(self.iw_interface)
                            self.iw_supports_scanning:bool = iwlib.utils.supports_scanning(self.iw_interface)
                        except OSError:
                            self.l.error(f'Error initiating interface {self.iw_interface}; wi-fi control disabled')
                            self.iw_enabled = False
                    elif dtype == DeviceType.MODEM:
                        self.l.info(f'NM: {self.iw_interface} type is CELLULAR {device_path}')
                        self.iw_device_type = IWStatus.DEVICE_TYPE_GSM
                        if not self.iw_modem_manager:
                            connection = Gio.bus_get_sync(Gio.BusType.SYSTEM, None)
                            self.iw_modem_manager = ModemManager.Manager.new_sync(
                                connection,
                                Gio.DBusObjectManagerClientFlags.DO_NOT_AUTO_START,
                                None) # MMModem('/org/freedesktop/ModemManager1/Modem/0', sdbus.get_default_bus())
                        objs = self.iw_modem_manager.get_objects()
                        for o in objs:
                            m = o.get_modem()
                            if m.get_primary_port() == self.iw_interface:
                                self.iw_modem_obj = o
                                self.iw_modem = m
                            
                    elif dtype == DeviceType.ETHERNET:
                        self.l.info(f'NM: {self.iw_interface} type is WIRED {device_path}')
                        self.iw_device_type = IWStatus.DEVICE_TYPE_WIRED
                    else:
                        self.l.error(f'NM: {self.iw_interface} type is UNKNOWN')
                        self.iw_enabled = False
        
        self.docker_cmd_srv = self.create_service(DockerCmd, f'/{self.node_name}/docker_command', self.docker_command_srv_callback)
        self.iw_scan_cmd_srv = self.create_service(IWScanCmd, f'/{self.node_name}/iw_scan', self.iw_scan_command_srv_callback)
            
        if self.docker_enabled:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.docker_pub = self.create_publisher(DockerStatus, self.docker_topic, qos)
            if self.docker_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.docker_topic}, msg_type=DockerStatus')
                self.docker_enabled = False
        
        if self.system_info_enabled:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.sysinfo_pub = self.create_publisher(SystemInfo, self.system_info_topic, qos)
            if self.sysinfo_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.system_info_topic}, msg_type=SystemInfo')
                self.system_info_enabled = False
                
        if self.iw_interface and self.iw_monitor_topic:
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.iw_pub = self.create_publisher(IWStatus, self.iw_monitor_topic, qos)
            if self.iw_pub == None:
                self.get_logger().error(f'Failed creating publisher for topic {self.iw_monitor_topic}, msg_type=IWStatus')
                self.iw_enabled = False
        
        file_extraction_signalling_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        self.file_request_sub = self.create_subscription(FileExtractionRequest, self.file_extraction_request_topic, self.file_request_received_callback, file_extraction_signalling_qos)
        self.file_result_pub = self.create_publisher(FileExtractionResult, self.file_extraction_result_topic, file_extraction_signalling_qos)
        
        file_chunks_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=100, reliability=QoSReliabilityPolicy.RELIABLE)
        if not self.bridge_server_address: # published chunks
            self.file_chunk_pub = self.create_publisher(FileChunk, self.file_extraction_chunks_topic, file_chunks_qos)
        else: # reads and uploads chunks from other agents
            self.file_chunk_sub = self.create_subscription(FileChunk, self.file_extraction_chunks_topic, self.file_chunk_received_callback, file_chunks_qos)
            self.file_chunks_receiving:dict[string, dict[int, bool]] = {}


    def file_request_received_callback(self, msg:FileExtractionRequest):
        
        search_path = msg.path
        
        res = FileExtractionResult()
        res.agent = self.node_name
        res.path = search_path
        res.id_robot = msg.id_robot
        
        if not self.file_extraction_enabled:
            res.result = FileExtractionResult.RESULT_EXTRACTION_DISABLED
            self.file_result_pub.publish(res)
            return
        if self.id_robot and msg.id_robot != self.id_robot:
            res.result = FileExtractionResult.RESULT_INVALID_ROBOT
            self.file_result_pub.publish(res)
            return # not for this agent
        
        self.l.info(f'File request received: {search_path}')
        
        # try to find the file
        file_bytes = locate_file(search_path, os.environ["ROS_DISTRO"], docker_client, self.l)
        
        if not file_bytes: # not found
            self.l.info(f'File {search_path} not found by {self.node_name}')
            res.result = FileExtractionResult.RESULT_NOT_FOUND
            self.file_result_pub.publish(res)
            return

        self.l.info(f'File {search_path} found by {self.node_name}')

        chunk_size = 500*1024 # ~0.5M is the default limit for topic message sizes without any extra config
        byte_size = len(file_bytes)
        num_parts = math.ceil(byte_size / chunk_size)

        if self.file_uploader_url_base: # upload it
            
            self.l.info(f'Uploading to {self.file_uploader_url_base}...')
            json_data = {
                "idRobot": self.id_robot,
                "key": self.key,
                "path": search_path,
                "parts": num_parts,
                "totalBytes": byte_size
            }
            if not upload_file_chunks(self.file_uploader_url_base, json_data, file_bytes, byte_size, chunk_size, self.l):
                res.result = FileExtractionResult.RESULT_ERROR
                self.file_result_pub.publish(res)
                return
            cached_file_name = complete_file_upload(self.file_uploader_url_base, json_data, self.l)
            if not cached_file_name:
                res.result = FileExtractionResult.RESULT_ERROR
                self.file_result_pub.publish(res)
                return
            res.result = FileExtractionResult.RESULT_UPLOADED
            res.cached_file_name = cached_file_name
        
        else: # can't upload from here, produce chunks into a topic and let other agent upload it
            
            self.l.info(f'Can\'t upload, producing {num_parts} file chunks...')
            res.result = FileExtractionResult.RESULT_FOUND_SENDING_CHUNKS
            produce_file_chunks(search_path, msg.id_robot, self.node_name, file_bytes, byte_size, chunk_size, num_parts, self.file_chunk_pub, self, self.l)

        self.file_result_pub.publish(res)


    def file_chunk_received_callback(self, msg:FileChunk):
        
        if self.id_robot != msg.id_robot:
            return
        
        self.l.debug(f'File chunk {msg.chunk_number+1}/{msg.total_chunks} of \'{msg.path}\' received from {msg.agent}')
        if msg.path not in self.file_chunks_receiving:
            self.file_chunks_receiving[msg.path] = {}
            for i in range(msg.total_chunks):
                self.file_chunks_receiving[msg.path][i] = False
            
        json_data = {
            "idRobot": self.id_robot,
            "key": self.key,
            "path": msg.path,
            "parts": msg.total_chunks,
            "totalBytes": msg.total_bytes
        }
        
        if not upload_file_chunk(self.file_uploader_url_base, json_data, msg.chunk_number, msg.data, self.l):
            res = FileExtractionResult()
            res.agent = self.node_name
            res.path = msg.path
            res.result = FileExtractionResult.RESULT_ERROR
            res.id_robot = msg.id_robot
            self.file_result_pub.publish(res)
            return
        
        self.file_chunks_receiving[msg.path][msg.chunk_number] = True
        
        all_done = True
        for i in range(msg.total_chunks):
            if not self.file_chunks_receiving[msg.path][i]:
                all_done = False
                break
        
        if all_done:
            self.l.info(f'All {msg.total_chunks} chunks done for \'{msg.path}\'')
            del self.file_chunks_receiving[msg.path]
            
            cached_file_name = complete_file_upload(self.file_uploader_url_base, json_data, self.l)
            res = FileExtractionResult()
            res.agent = self.node_name
            res.path = msg.path
            res.id_robot = msg.id_robot
            if not cached_file_name:
                res.result = FileExtractionResult.RESULT_ERROR
                self.file_result_pub.publish(res)
                return
            res.result = FileExtractionResult.RESULT_UPLOADED
            res.cached_file_name = cached_file_name
            self.file_result_pub.publish(res)


    def docker_command_srv_callback(self, request:DockerCmd.Request, response:DockerCmd.Response):
        response.err = 0
        
        if not self.docker_control_enabled:
            response.err = 3
            response.msg = 'Docker control disabled'
            return response
        
        self.get_logger().debug(f'Docker request {request.id_container} state:{request.set_state}')
        
        if not request.id_container:
            response.err = 3
            response.msg = 'No container id provided'
            return response
        
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
                        response.msg = f'Container {request.id_container} already running'
                        return response
                    cont.start()
                    response.msg = f'Container {request.id_container} starting...'
                case 0:
                    if cont.status == 'exited':
                        response.err = 3
                        response.msg = f'Container {request.id_container} already exited'
                        return response
                    cont.stop(timeout=3)
                    response.msg = f'Container {request.id_container} stopping...'
                case 2:
                    if cont.status == 'restarting':
                        response.err = 3
                        response.msg = f'Container {request.id_container} already restarting'
                        return response
                    cont.restart(timeout=3)
                    response.msg = f'Container {request.id_container} restarting...'
        except Exception as e:
            response.err = 3
            response.msg = f'Docker exception: {str(e)}'
            return response
        
        return response


    def iw_scan_command_srv_callback(self, request:IWScanCmd.Request, response:IWScanCmd.Response):
        response.err = 0
        
        self.get_logger().info(f'IW scan request received; roam={request.attempt_roam}')
                
        if not self.iw_supports_scanning:
            response.err = 3
            response.msg = 'Interface doesn\'t support scanning'
            return response

        if not self.iw_control_enabled:
            response.err = 3
            response.msg = 'Wifi control disabled by Agent'
            return response
        
        try:
            results = subprocess.run(['iw', 'dev', self.iw_interface, 'scan'], capture_output=True, text=True)
            print(results.stdout)
            # results = iwlib.iwlist.scan(self.iw_interface)
        except Exception as e:
             print(f'Exception while scanning IW: {e}')
             response.err = 3
             response.msg = f'Exception while scanning: {str(e)}'
             return response

        self.get_logger().info(f'IW Monitor scan results: ')
        
        response.scan_results = []
        roaming_candidates = []

        curr_res = None
        
        for l in results.stdout.splitlines():
            
            if l.startswith('BSS'):
                curr_res = IWScanResult()
                response.scan_results.append(curr_res)
                parts = l.split(' ')
                if len(parts) > 0:
                    bss_parts = parts[1].split('(')
                    if len(bss_parts) > 0:
                        curr_res.access_point = bss_parts[0].strip().lower()
                curr_res.current = 'associated' in l                    
                continue              
            
            if curr_res == None:
                continue
            
            l = l.strip()
            
            if l.lower().startswith('freq'):
                parts = l.split(':')
                if len(parts) > 0:
                    float_freq = float(parts[1].strip())
                    curr_res.frequency = float_freq / 1000.0
                continue
            
            if l.lower().startswith('ssid'):
                parts = l.split(':')
                if len(parts) > 0:
                    curr_res.essid = parts[1].strip()
                    if curr_res.essid.lower() == self.last_essid.lower():
                        curr_res.roaming_candidate = True
                        roaming_candidates.append(curr_res)
                continue
            
            if l.lower().startswith('signal'):
                parts = l.split(':')
                if len(parts) > 0:
                    curr_res.signal = float(parts[1].replace('dBm', '').strip())
                continue
        
        response.scan_results = sorted(response.scan_results, key=lambda x: x.signal, reverse=True)
        
        if request.attempt_roam:
            if not self.iw_roaming_enabled:
                response.err = 3
                response.msg = 'Roaming disabled by Agent'
            else:
                self.get_logger().info(f'IW: Seeing {len(roaming_candidates)} roaming candidates for {self.last_essid}')
                
                roaming_candidates = sorted(roaming_candidates, key=lambda x: x.signal, reverse=True)        
                bestest = roaming_candidates[0]
                
                if bestest.access_point.lower() == self.last_access_point.lower():
                    response.res = 0
                    response.msg = 'Not roaming, current AP seems the best'
                    self.get_logger().info(f" >>> Not roaming, current AP seems the best")
                else:
                    self.get_logger().info(f' >>> Attenmpting to roam to "{bestest.essid}" {bestest.access_point} with signal={bestest.signal}')
                    wpa_cli_res = os.system(f'wpa_cli -p /host_run/wpa_supplicant/ -i {self.iw_interface} roam {bestest.access_point}')
                    self.get_logger().info(f'wpa_cli_res={wpa_cli_res}')
                    response.res = wpa_cli_res
                    response.msg = f'Switched to "{bestest.essid}" {bestest.access_point}'

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
            'mem': 0, # TODO ignoring for now, mem not working on linux
            'mem_perc': 0.0,
            'net_read': 0, # TODO ignoring for now, net not working on linux
            'net_write': 0,
        }
        
        if 'pids_stats' in stats and 'current' in stats['pids_stats']:
            res['pids'] = stats['pids_stats']['current']
        
        if 'cpu_stats' in stats and 'system_cpu_usage' in stats['cpu_stats'] \
        and 'precpu_stats' in stats and 'system_cpu_usage' in stats['precpu_stats']:
            cpu_delta = (stats['cpu_stats']['cpu_usage']['total_usage']
                        - stats['precpu_stats']['cpu_usage']['total_usage'])
            system_delta = (stats['cpu_stats']['system_cpu_usage']                    
                        - stats['precpu_stats']['system_cpu_usage'])
            res['num_cpus'] = stats['cpu_stats']["online_cpus"]
            res['cpu_perc'] = (cpu_delta / system_delta) * res['num_cpus'] * 100.0
            res['cpu_max_perc'] = res['num_cpus'] * 100

        if 'blkio_stats' in stats and 'io_service_bytes_recursive' in stats['blkio_stats']\
        and stats['blkio_stats']['io_service_bytes_recursive'] != None:
            for blkio_stats in stats['blkio_stats']['io_service_bytes_recursive']:
                if blkio_stats['op'] == 'read':
                    res['block_io_read'] = blkio_stats['value']
                elif blkio_stats['op'] == 'write':
                    res['block_io_write'] = blkio_stats['value']
        return res


    def get_docker_containers(self):
        
        if not self.docker_pub or not self.context.ok():
            if self.shutting_down:
                print('Ignoring pushing docker state after shutdown')  
            return
        
        if not docker_client:
            return
        
        docker_containers = docker_client.containers.list(all=True)
         
        msg = DockerStatus()
        set_message_header(self, msg)
        msg.containers = []
         
        c_stats = []
        for cont in docker_containers:
            msg_cont = DockerContainerStatus()
            cs = {}
            if cont.status == 'running':
                stats = cont.stats(stream=False, decode=False) # stream returns wrong data, don't use 
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
            if self.shutting_down:
                cs['status'] = 'exited'
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
            
            if self.log_output:
                if cs['status'] == 'running':
                    print(f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}] CPU: {cs["cpu_perc"]:.2f}% BLOCK I/O: {format_bytes(cs["block_io_read"], True)} / {format_bytes(cs["block_io_write"], True)} PIDS: {str(cs["pids"])}')
                else:
                    print(f'[Docker] {cs["short_id"]} {c(cs["name"], clr)} [{c(cs["status"], clr)}]')
        
        if self.docker_pub and self.context.ok():
            self.docker_pub.publish(msg)
        elif self.shutting_down:
          print('Error pushing docker state after shutdown')  


    def get_system_info(self):
        cpu_count = psutil.cpu_count()
        cpu_times = psutil.cpu_times_percent(interval=1, percpu=True)
        mem = psutil.virtual_memory()
        swp = psutil.swap_memory()
        
        msg = SystemInfo()
        set_message_header(self, msg)
        
        msg.cpu = []
        i = 0
        for cpu in cpu_times:
            if self.log_output:
                print(f'[CPU {str(i)}] User:{cpu.user:.1f}% Nice:{cpu.nice:.1f}% Sys:{cpu.system:.1f}% Idle:{cpu.idle:.1f}% ... {100.0-cpu.idle:.1f}%')
            i += 1
            msg_cpu = CPUStatusInfo()
            msg_cpu.user_percent = cpu.user
            msg_cpu.nice_percent = cpu.nice
            msg_cpu.system_percent = cpu.system
            msg_cpu.idle_percent = cpu.idle
            msg.cpu.append(msg_cpu)
        
        if self.log_output:
            print(f'[MEM] Tot:{format_bytes(mem.total)} Avail:{format_bytes(mem.available)} Used:{format_bytes(mem.used)} Free:{format_bytes(mem.free)} Buff:{format_bytes(mem.buffers)} Shar:{format_bytes(mem.shared)} Cach:{format_bytes(mem.cached)}')
        msg.mem_total_bytes = mem.total
        msg.mem_available_bytes = mem.available
        msg.mem_used_bytes = mem.used
        msg.mem_free_bytes = mem.free
        msg.mem_buffers_bytes = mem.buffers
        msg.mem_shared_bytes = mem.shared
        msg.mem_cached_bytes = mem.cached

        if self.log_output:
            print(f'[SWP] Tot:{format_bytes(swp.total)} Used:{format_bytes(swp.used)} Free:{format_bytes(swp.free)}')
        msg.swp_total_bytes = swp.total
        msg.swp_used_bytes = swp.used
        msg.swp_free_bytes = swp.free
        
        i = 0
        msg.disk = []
        for disk_path in self.disk_paths:
            dsk = psutil.disk_usage(disk_path)
            i += 1
            if self.log_output:
                print(f'[DSK {disk_path}] Tot:{format_bytes(dsk.total)} Used:{format_bytes(dsk.used)} Free:{format_bytes(dsk.free)}')    
            msg_dsk = DiskVolumeStatusInfo()
            msg_dsk.path = disk_path
            msg_dsk.total_bytes = dsk.total
            msg_dsk.used_bytes = dsk.used
            msg_dsk.free_bytes = dsk.free
            msg.disk.append(msg_dsk)
        
        if self.sysinfo_pub and self.context.ok():
            self.sysinfo_pub.publish(msg)


    def get_wifi_connection_info(self):
        
        cfg = iwlib.iwconfig.get_iwconfig(self.iw_interface)
        msg = IWStatus()
        msg.device_type = self.iw_device_type
        
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

            if self.log_output:
                print(f'[WIFI] Q:{str(msg.quality)}% L:{str(msg.level)} N:{str(msg.noise)} AP:{msg.access_point}')
        
            if self.iw_pub and self.context.ok():
                self.iw_pub.publish(msg)

        except Exception as e:
            print (f'Error while generating IWStatus: {e}')
            print (f'IW CFG was: {cfg}')


    async def get_gsm_connection_info(self):
            
            if not self.iw_modem_obj or not self.iw_modem:
                return
            
            msg = IWStatus()
            msg.device_type = self.iw_device_type
            msg.supports_scanning = False
            
            set_message_header(self, msg)

            # 3GPP interface for operator name
            modem_3gpp = self.iw_modem_obj.get_modem_3gpp()
            operator_name = modem_3gpp.get_operator_name() if modem_3gpp else "unknown"

            # Signal quality (0–100)
            signal_quality, _recent = self.iw_modem.get_signal_quality()

            access_tech_flags = self.iw_modem.get_access_technologies()
            using_techs = []
                        
            for tech in dir(ModemManager.ModemAccessTechnology):
                if tech.startswith("_"):
                    continue
                flag = getattr(ModemManager.ModemAccessTechnology, tech)
                if not isinstance(flag, int):
                    continue
                if flag == ModemManager.ModemAccessTechnology.ANY:
                    continue
                if access_tech_flags & flag:
                    using_techs.append(tech)
                            
            msg.gsm_tech = ", ".join(using_techs) if using_techs else ""
            msg.access_point = operator_name
            
            # print(f"{operator_name} {signal_quality} {connection_mode}")
            msg.quality_max = 100
            msg.quality = signal_quality
            
            if self.log_output:
                print(f'[GSM] {operator_name} Q:{str(msg.quality)}% {msg.gsm_tech}')
            
            try:
                if self.iw_pub and self.context.ok():
                    self.iw_pub.publish(msg)
            except Exception as e:
                print (f'Error while generating GSM IWStatus: {e}')


    def get_wired_connection_info(self):
        
        msg = IWStatus()
        msg.device_type = self.iw_device_type
        msg.supports_scanning = False
        
        set_message_header(self, msg)

        msg.quality_max = 100
        msg.quality = 100
            
        try:
            if self.iw_pub and self.context.ok():
                self.iw_pub.publish(msg)
        except Exception as e:
            print (f'Error while generating Wired IWStatus: {e}')


    async def agent_loop(self):

        try:
            while not self.shutting_down:
                
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.docker_enabled and (not self.docker_task or self.docker_task.done()):
                    self.docker_task = asyncio.get_event_loop().run_in_executor(None, self.get_docker_containers)
                    
                if self.system_info_enabled and (not self.sysinfo_task or self.sysinfo_task.done()):
                    self.sysinfo_task =  asyncio.get_event_loop().run_in_executor(None, self.get_system_info)
                    
                if self.iw_enabled and (not self.iw_task or self.iw_task.done()):
                    if self.iw_device_type == IWStatus.DEVICE_TYPE_WIFI:
                        self.iw_task =  asyncio.get_event_loop().run_in_executor(None, self.get_wifi_connection_info)
                    elif self.iw_device_type == IWStatus.DEVICE_TYPE_GSM:
                        self.iw_task =  asyncio.get_event_loop().create_task(self.get_gsm_connection_info())
                    elif self.iw_device_type == IWStatus.DEVICE_TYPE_WIRED:
                        self.iw_task =  asyncio.get_event_loop().run_in_executor(None, self.get_wired_connection_info)
                await asyncio.sleep(self.refresh_period_sec)
            
        except (asyncio.CancelledError, KeyboardInterrupt):
            pass
        except Exception as e:
            self.get_logger().error(f'Exception in agent_loop: {e}')
        
        self.get_logger().debug(f'Loop stopped')


    def load_config(self):
        
        self.declare_parameter('agent_log_verbose', False)
        self.log_output = self.get_parameter('agent_log_verbose').get_parameter_value().bool_value
        
        self.declare_parameter('agent_update_period_sec', 0.5)
        self.refresh_period_sec = self.get_parameter('agent_update_period_sec').get_parameter_value().double_value
        self.get_logger().info(f'Refresh period is {self.refresh_period_sec:.1f}s')
        
        self.declare_parameter('docker_monitor_topic', '/docker_info')
        self.docker_topic = self.get_parameter('docker_monitor_topic').get_parameter_value().string_value
        self.docker_enabled = self.docker_topic != ''
        if self.docker_enabled:
            self.get_logger().info(f'Monitoring Docker -> {self.docker_topic}')
            
        self.declare_parameter('enable_docker_control', True)
        self.docker_control_enabled = self.get_parameter('enable_docker_control').get_parameter_value().bool_value
        if self.docker_control_enabled:
            self.get_logger().info(f'Docker control enabled')
        
        self.declare_parameter('system_info_topic', '/system_info')
        self.system_info_topic = self.get_parameter('system_info_topic').get_parameter_value().string_value
        self.system_info_enabled = self.system_info_topic != ''
        if self.system_info_enabled:
            self.get_logger().info(f'System monitoring CPU/MEM/SWP+disks -> {self.system_info_topic}')
      
        self.declare_parameter('disk_volume_paths', [ '/' ]) 
        self.disk_paths = self.get_parameter('disk_volume_paths').get_parameter_value().string_array_value
        if self.system_info_enabled:
            self.get_logger().info(f'Monitoring disk volumes: {str(self.disk_paths)}')
            
        self.declare_parameter('wifi_interface', '')
        self.iw_interface = self.get_parameter('wifi_interface').get_parameter_value().string_value
        self.declare_parameter('wifi_monitor_topic', '/iw_status')
        self.iw_monitor_topic = self.get_parameter('wifi_monitor_topic').get_parameter_value().string_value
        self.iw_enabled = self.iw_interface and self.iw_monitor_topic
        if self.iw_enabled:
            self.get_logger().info(f'Monitoring network interface {self.iw_interface} -> {self.iw_monitor_topic}')

        self.declare_parameter('enable_wifi_scan', True)
        self.iw_control_enabled = self.get_parameter('enable_wifi_scan').get_parameter_value().bool_value
        self.declare_parameter('enable_wifi_roam', False)
        self.iw_roaming_enabled = self.get_parameter('enable_wifi_roam').get_parameter_value().bool_value
        if self.iw_enabled and self.iw_control_enabled:
            self.get_logger().info(f'Network control enabled'+(' with roaming' if self.iw_roaming_enabled else ''))
            
        self.declare_parameter('file_extraction_enabled', True)
        self.file_extraction_enabled = self.get_parameter('file_extraction_enabled').get_parameter_value().bool_value
        if self.file_extraction_enabled:
            self.get_logger().info(f'File extraction enabled')
        else:
            self.get_logger().info(f'File extraction disabled')
        
        self.declare_parameter('id_robot', '')
        self.id_robot = self.get_parameter('id_robot').get_parameter_value().string_value
        self.declare_parameter('key', '')
        self.key = self.get_parameter('key').get_parameter_value().string_value
        self.declare_parameter('bridge_server_address', '')
        self.bridge_server_address = self.get_parameter('bridge_server_address').get_parameter_value().string_value
        self.declare_parameter('file_uploader_port', 1336)
        self.file_uploader_port = self.get_parameter('file_uploader_port').get_parameter_value().integer_value
        self.file_uploader_url_base = None
        if self.id_robot and self.key and self.bridge_server_address:
            self.file_uploader_url_base = f'{self.bridge_server_address}:{self.file_uploader_port}'
            self.get_logger().info(f'File upload enabled ({self.file_uploader_url_base})')
        else:
            self.get_logger().info(f'File upload disabled')
         
        self.declare_parameter('file_extraction_request_topic', '/file_extraction_requests')
        self.file_extraction_request_topic = self.get_parameter('file_extraction_request_topic').get_parameter_value().string_value
        self.declare_parameter('file_extraction_result_topic', '/file_extraction_results')
        self.file_extraction_result_topic = self.get_parameter('file_extraction_result_topic').get_parameter_value().string_value
        self.declare_parameter('file_extraction_chunks_topic', '/file_extractor_chunks')
        self.file_extraction_chunks_topic = self.get_parameter('file_extraction_chunks_topic').get_parameter_value().string_value
        

    async def shutdown_cleanup(self):
        
        if self.docker_pub:
            print(f'Pushing shutdown state docker containers...')
            self.get_docker_containers()
            # await asyncio.sleep(3)
            self.docker_pub.destroy()
            self.docker_pub = None
            
        if self.sysinfo_pub:
            self.sysinfo_pub.destroy()
            self.sysinfo_pub = None
            
        if self.iw_pub:
            self.iw_pub.destroy()
            self.iw_pub = None


async def main_async(args):
    agent_node = None
    loop_task = None
    try:
        agent_node = AgentController()
        await agent_node.setup()
        loop_task = asyncio.get_event_loop().create_task(agent_node.agent_loop())
        await asyncio.wait([ loop_task ], return_when=asyncio.ALL_COMPLETED)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    except Exception as e:
        print(f'Exception in main_async(): {e}')
        traceback.print_exc(e)
    
    print('SHUTTING DOWN')
    
    agent_node.shutting_down = True
    
    if loop_task != None and not loop_task.done():
        loop_task.cancel()
    
    await agent_node.shutdown_cleanup()
    try:
        agent_node.destroy_node()
    except:
        pass


class MyAsyncioPolicy(asyncio.DefaultEventLoopPolicy):
    def new_event_loop(self):
        selector = selectors.SelectSelector()
        return asyncio.SelectorEventLoop(selector)


def main(args=None): # ros2 calls this, so init here
    rclpy.init()
    asyncio.set_event_loop_policy(MyAsyncioPolicy())
    try:
        asyncio.run(main_async(args))
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    try:
        rclpy.shutdown()
    except:
        pass


if __name__ == '__main__':
    main()
