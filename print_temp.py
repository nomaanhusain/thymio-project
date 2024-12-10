import os
import glob
import time

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

f = open(device_file, 'r')
lines = f.readlines()
f.close()

equals_pos = lines[1].find('t=')
temp_c = -273
if equals_pos != -1:
    temp_string = lines[1][equals_pos + 2:]
    temp_c = float(temp_string) / 1000.0

print(f"temp_c = {temp_c}")
