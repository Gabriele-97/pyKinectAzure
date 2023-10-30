import cv2

import pykinect_azure as pykinect
import time
import statistics
import numpy as np
import csv

if __name__ == "__main__":
	foldername = 'Azure_Validation'
	print('Input the name of the file')
	filename = input()
	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries(track_body=True)

	# Modify camera configuration
	device_config = pykinect.default_configuration
	# print(device_config)
	device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED # WE CAN TRY TO CHANGE MODE TO WFOV
	#print(device_config)

	# Start device
	filepath = 'c://Users//laura//Documents//pyKinectAzure//' + foldername + '//'
	video_path = filepath + filename + '_video.avi'

	device = pykinect.start_device(config=device_config,  record=True, record_filepath=video_path)

	# Start body tracker
	bodyTracker = pykinect.start_body_tracker()

	cv2.namedWindow('Depth image with skeleton',cv2.WINDOW_NORMAL)
	tempi = []
	imagebuffer = []
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
				print(skeleton[0][0])
				if skeleton[0][0] > 0:
					subj = 0
				else:
					subj = 1
			else:
				subj = 0

			stampstring = str(time.time()) + " " + str(subj) + " " +" ".join(str(r) for v in skeleton for r in v) + "\n"
			with open('c://Users//laura//Documents//pyKinectAzure//'+ foldername + '//' + filename + str(tempi[0]) + '.txt','a') as f:
				f.write(stampstring)

		
		# with open(filename +'.csv',  'a', newline='') as file:
		# 	writer = csv.writer(file)		
		# 	writer.writerow(color_image)

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
	 


