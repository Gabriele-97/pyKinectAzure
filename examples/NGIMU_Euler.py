#!/usr/bin/python3

import osc_decoder
import socket
import time
import numpy as np
import math
import csv
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
from numpy.linalg import norm
from datetime import datetime
from pythonosc import udp_client
import json
import os  


ignore_magnetometer=1
# put =1 in case of acquisitions in noisy environment--> NB: align IMUs to each other before running the script!

# PC Public IP address. This works only if the operating system is Linux
# for name, interface in ifcfg.interfaces().items():
#    if interface['device'] == "wlan0":      # Device name
#        IPAddr = interface['inet']          # First IPv4 found
#        print("You are connected to the network. IP Address: ", IPAddr)    

IPAddr = "192.168.0.106" #Put here IP address of the PC when connected to the NGIMU network     

# These are the IP addresses of each IMU. They are used to send commands to the IMUs
# You can find and modify them with the GUI. Please make sure they are correct 
send_addresses = ["192.168.0.104"] 

# The send port is the same for each IMU and can be found in the GUI as the receive port of the IMU 
# The send port is the one that the IMUs listen to 
send_port = 9000

# Array of UDP ports to listen to, one per NGIMU.  These ports must be equal to
# the UDP Send Port in the NGIMU settings. This setting is changed
# automatically when connecting to the NGIMU using the NGIMU GUI. Please make sure they are correct.
receive_ports = [8104]

# IMUs desired send rate [Hz]
send_rate = 100 
    
# Send /identify message to strobe all LEDs.  The OSC message is constructed
# from raw bytes as per the OSC specification.  The IP address must be equal to
# the IP address of the target NGIMU. The port must be equal to the Receive Port
# in the NGIMU UDP settings
print("Opening UDP socket...")
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Creation of the .csv file 
current_datetime = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
str_current_datetime = str(current_datetime)
name_euler = "IMU_data_euler"+str_current_datetime+".csv"
name_acc = "IMU_data_acc"+str_current_datetime+".csv"
outdir = 'c://Users//laura//Documents//pyKinectAzure//FEV_lab_gruppo6'
#outdir = './DATA'
if not os.path.exists(outdir):
     os.mkdir(outdir)
name_eulerfile = os.path.join(outdir, name_euler) 
name_accfile = os.path.join(outdir, name_acc) 
euler = open(name_eulerfile,'w',encoding='UTF8')
acc = open(name_accfile,'w',encoding='UTF8')
writer_euler = csv.writer(euler, delimiter=',')
writer_acc = csv.writer(acc, delimiter=',')
header_euler = ['sys_time','imu_time_stamp','roll','pitch','yaw']
header_acc = ['sys_time','imu_time_stamp','a_x','a_y','a_z']
writer_euler.writerow(header_euler)
writer_acc.writerow(header_acc)

for send_address in send_addresses:
    IMU_client = udp_client.SimpleUDPClient(send_address, send_port)
    # Make the led blink
    IMU_client.send_message("/identify", 0.0)
    # Change IMUs UDP send addresses to match PC IP address
    IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the PC when connected to the network
    # Set the IMUs to send euler angles and linear acceleration at a certain frequency
    IMU_client.send_message("/rate/euler", send_rate)
    IMU_client.send_message("/rate/linear", send_rate)
    # Ignore/Use magnetometer
    if ignore_magnetometer==1:
        IMU_client.send_message("/ahrs/magnetometer", True)
    else:
        IMU_client.send_message("/ahrs/magnetometer", False)

    print("Updating IMU settings...")
    #time.sleep(10) --> this might be needed to give time to the IMU to update the settings

# Open the UDP connection to continuously read messages from the IMUs network
receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

index = 0
for receive_socket in receive_sockets:
    receive_socket.bind(("", receive_ports[index]))
    index = index + 1
    receive_socket.setblocking(False)
    
print("Starting communication...")

timecount=0
t0 = time.time()
read = False

while True:
    for udp_socket in receive_sockets: 
        time_stamp = 0
        data_type = 0 
        sys_time = time.time()-t0
        try:
            data, addr = udp_socket.recvfrom(2048)
            
        except socket.error:
            pass
        else:
            for message in osc_decoder.decode(data):
                #print(message)  
                              
                time_stamp = message[0]
                data_type = message[1]    
                if read == False:
                    t_stamp0 = time_stamp          
                if data_type == '/euler': #this can be changed with every message avaible from the IMUs (gyro data, temperature...)
                    read = True
                    roll = message[2]
                    pitch = message[3]
                    yaw = message[4]
                    data = [sys_time, time_stamp-t_stamp0, roll, pitch, yaw]  
                    writer_euler.writerow(data)
                if data_type == '/linear': #this can be changed with every message avaible from the IMUs (gyro data, temperature...)
                    read = True
                    a_x = message[2]
                    a_y = message[3]
                    a_z = message[4]
                    data = [sys_time, time_stamp-t_stamp0, a_x, a_y, a_z]
                    writer_acc.writerow(data)         
            
            if timecount%100==0:
                print("Roll: ", roll)                 
                print("Pitch: ", pitch)                
                print("Yaw: ", yaw)
                print("acc x: ", a_x)                 
                print("acc y: ", a_y)                
                print("acc z: ", a_z)

            timecount = timecount+1
                
    
        