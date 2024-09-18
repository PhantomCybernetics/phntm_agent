import sys
import time
import math


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

def print_line(node, str):
    print(str)

def set_message_header(node, msg):
    time_nanosec:int = time.time_ns()
    msg.header.stamp.sec = math.floor(time_nanosec / 1000000000)
    msg.header.stamp.nanosec = time_nanosec % 1000000000
    msg.header.frame_id = node.hostname