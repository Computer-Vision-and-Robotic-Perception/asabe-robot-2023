# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2 as cv

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

#introduce stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipe_profile = pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()


# Intrinsics & Extrinsics
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
#print(depth_intrin)
# width: 640, height: 480, ppx: 321.388, ppy: 240.675, fx: 385.526, fy: 385.526, model: Brown Conrady, coeffs: [0, 0, 0, 0, 0]
color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
dist_to_center = depth_frame.get_distance(320, 240)
depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " ,depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 0.4572#1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)

path = '/home/asabe/Desktop/realsense/data/images'

try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()


        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        #grey_color = 153
        grey_color = 0
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        #clipping_distance = 0    #added 05312022
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        img = bg_removed
        org = bg_removed
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        low_green = np.array([50,86,40])
        high_green = np.array([100,255,255])
        low_brown = np.array([3,70,20])
        high_brown = np.array([20,180,80])

        brownMask = cv.inRange(hsv, low_brown, high_brown)
        greenMask = cv.inRange(hsv, low_green, high_green)
        Mask =  brownMask | greenMask
        src = cv.bitwise_and(img, img, mask = Mask)

        img=cv.subtract(img, src)
        #cv.imshow('original', img)

        kernel = np.ones( (20 , 20) , np.uint8)
        binary = cv.erode(img,kernel,iterations = 1)
        kernel2 = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        binary = cv.erode(binary, kernel2, iterations=2)
        kernel = np.ones( (8 , 8) , np.uint8)
        binary = cv.erode(binary,kernel,iterations = 1)
	gray = cv.cvtColor(binary, cv.COLOR_BGR2GRAY)
        #cv.imshow('oginal', binary)

        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(gray, connectivity=8)
        output = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        for i in range(1, num_labels):

            mask = labels == i
            output[:, :, 0][mask] = np.random.randint(0, 255)
            output[:, :, 1][mask] = np.random.randint(0, 255)
            output[:, :, 2][mask] = np.random.randint(0, 255)
        #cv.imshow('oginal', output)

        vs1 = np.hstack((org, img))  
        vs2 = np.hstack((binary, output))  
        result = np.vstack((vs1, vs2))
        cv.imshow("result", result)
        
        #print('stats = ',stats)
	print('num_labels = ',num_labels)
	
	x = np.empty([num_labels,1], dtype = int)
	centroids=np.c_[centroids, x.T]

	x1 = np.delete(centroids, 0, axis=0)
	x1=np.int_(x1)
	print(x1, x1.shape)
	# 每一个像素的标签1、2、3.。。，同一个连通域的标签是一致的centroids, centroids.shape
	# print('labels = ',labels)
	i=0

	while i<(num_labels-1):
	    x=centroids[i][0]
	    y=centroids[i][1]
	    centroids[i][2]=vtx[y][x][0][2]
	    i+=1
        print(centroids)
	
        k = cv.waitKey(1)
        # Press esc or 'q' to close the image window
        if k & 0xFF == ord('q') or k == 27:
            cv.destroyAllWindows()
            break
	#capture image if space bar is pressed
	if k%256 == 32:
		imgName1 = "Original_{}.png".format(img_counter)
                imgName2 = "Removed_{}.png".format(img_counter)
		cv.imwrite(os.path.join(path,imgName1), color_image)
                cv.imwrite(os.path.join(path,imgName2), bg_removed)
		img_counter = img_counter + 1
finally:
    pipeline.stop()
