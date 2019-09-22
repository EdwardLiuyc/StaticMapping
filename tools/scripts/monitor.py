import os
import time
import psutil

from subprocess import check_output


def get_pid(name):
    return check_output(["pidof", name])


process_name = "static_mapping_node"
pid = get_pid(process_name)
process = psutil.Process(int(pid))
a = 0
while(a == 0):
    mem = process.memory_info().rss
    mem /= (1024*1024)  # MB
    print(mem)  # in bytes
    time.sleep(1)
