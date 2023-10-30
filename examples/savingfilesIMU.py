import cv2

import pykinect_azure as pykinect
import time
import statistics
import numpy as np

import osc_decoder
import socket
from datetime import datetime
from pythonosc import udp_client
import os  
import csv

import threading

current_datetime = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
str_current_datetime = str(current_datetime)
t0 = time.time()

class IMUReadLoop(threading.Thread):
	def __init__(self,name,event):
		threading.Thread.__init__(self)
		self.name = name
		self.event = event

	def run(self):		
		# IMUs settings
		IPAddr = "192.168.0.105" #Put here IP address of the PC when connected to the NGIMU network     

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
		
		name_euler = "IMU_data_euler"+str_current_datetime+".csv"
		name_acc = "IMU_data_acc"+str_current_datetime+".csv"
		outdir = 'c://Users//laura//Documents//pyKinectAzure//FEV_lab_gruppo6'
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
			# IMU_client.send_message("/wifi/send/ip", IPAddr) #IP address of the PC when connected to the network
			# Set the IMUs to send euler angles and linear acceleration at a certain frequency
			IMU_client.send_message("/rate/euler", send_rate)
			IMU_client.send_message("/rate/linear", send_rate)
			IMU_client.send_message("/ahrs/magnetometer", True)
			


			print("Updating IMU settings...")
			#time.sleep(10) --> this might be needed to give time to the IMU to update the settings

		# Open the UDP connection to continuously read messages from the IMUs network
		receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

		index = 0
		for receive_socket in receive_sockets:
			receive_socket.bind(("", receive_ports[index]))
			index = index + 1
			receive_socket.setblocking(True)
			
		print("Starting communication...")

		
		read = False
		timecount=0
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
			if self.event.is_set():
				print("Exiting IMU thread")
				break

class KinectReadLoop(threading.Thread):
	def __init__(self, name, event):
		threading.Thread.__init__(self)
		self.name = name
		self.event = event
	def run(self):
		# Initialize the library, if the library is not found, add the library path as argument
		pykinect.initialize_libraries(track_body=True)

		# Modify camera configuration
		device_config = pykinect.default_configuration
		# print(device_config)
		device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_OFF
		device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
		#print(device_config)

		# Start device
		device = pykinect.start_device(config=device_config)

		# Start body tracker
		bodyTracker = pykinect.start_body_tracker()

		cv2.namedWindow('Depth image with skeleton',cv2.WINDOW_NORMAL)
		tempi = []

		while True:
			# Get capture
			capture = device.update()

			# Get body tracker frame
			body_frame = bodyTracker.update()

			# Get the color depth image from the capture
			ret_depth, depth_color_image = capture.get_colored_depth_image()
			ret_color, color_image = capture.get_color_image()


			# Get the colored body segmentation
			ret_color, body_image_color = body_frame.get_segmentation_image()
			tempi.append(time.time())


			if not ret_depth or not ret_color:
				continue

			for body_id in range(body_frame.get_num_bodies()):
				bodies = body_frame.get_body2d(body_id,pykinect.K4A_CALIBRATION_TYPE_DEPTH).numpy()
				skeleton = body_frame.get_body(body_id).numpy()[:, :3]
				if body_frame.get_num_bodies() != 0:
					#print(skeleton[0][0])
					if skeleton[0][0] > 0:
						subj = 0
					else:
						subj = 1
				else:
					subj = 0

				stampstring = str(time.time()-t0) + " " + str(subj) + " " +" ".join(str(r) for v in skeleton for r in v) + "\n"
				with open('c://Users//laura//Documents//pyKinectAzure//FEV_lab_gruppo6//kinect_data'+str_current_datetime +'.txt','a') as f:
					f.write(stampstring)

				
			# Combine both images
			try:
				color_image_rsz = cv2.resize(color_image,(640,576))
			except:
				pass
			combined_image = cv2.addWeighted(depth_color_image, 1, body_image_color, 0.4, 0)
			# Draw the skeletons
			combined_image = body_frame.draw_bodies(combined_image)

			# Overlay body segmentation on depth image
			cv2.imshow('Depth image with skeleton',combined_image)

			# Press q key to stop
			if cv2.waitKey(1) == ord('q'):
				break

			if self.event.is_set():
				print("Exiting Kinect thread")
				break
		
		period =[]
		fps =[]
		for i in range(1,len(tempi),1):
			period.append(tempi[i]-tempi[i-1])
			fps.append(1/period[-1])
		mediat = statistics.mean(period)
		stdt = statistics.stdev(period)
		mediafps = statistics.mean(fps)
		stdfps = statistics.stdev(fps)
		print('mean period:', mediat)
		print('std period:', stdt)
		print('mean fps:', mediafps)
		print('std fps:', stdfps)		
			

	
if __name__ == "__main__":
	event = threading.Event()
	KinectReadThread = KinectReadLoop("Kinect read",event)
	IMUReadThread = IMUReadLoop("IMU Read",event)

	threads = []
	threads.append(KinectReadThread)
	threads.append(IMUReadThread)

	for t in threads:
		t.start()

	while True:
		try:
			time.sleep(0.05)
		except KeyboardInterrupt:
			event.set()
			break

	#for t in threads:
	#	t.join()

	
