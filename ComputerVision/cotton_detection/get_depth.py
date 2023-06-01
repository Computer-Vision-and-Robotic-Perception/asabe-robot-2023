import pyrealsense2 as rs
import numpy as np
pipeline = rs.pipeline()
 
config = rs.config()
 
# This is the minimal recommended resolution for D435
 
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
 
pipe_profile = pipeline.start(config)
 
pc = rs.pointcloud()
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()
 
print(depth)
 
img_color = np.asanyarray(color.get_data())
img_depth = np.asanyarray(depth.get_data())
pc.map_to(color)
points = pc.calculate(depth)
#
vtx = np.asanyarray(points.get_vertices())
vtx = np.reshape(vtx, (480, 640, -1))
print(vtx[240][320])
