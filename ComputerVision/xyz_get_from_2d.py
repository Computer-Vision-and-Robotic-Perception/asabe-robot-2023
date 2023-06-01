# -*- coding: utf-8 -*-
# encoding: utf-8print
import pyrealsense2 as rs
import numpy as np
import cv2
 
''' 
开启点云
'''
# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()
 
''' 
设置
'''
 # 定义流程pipeline，创建一个管道
pipeline = rs.pipeline()

# 定义配置config
config = rs.config()
# 颜色和深度流的不同分辨率
# 配置depth流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
# config.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
# config.enable_stream(rs.stream.depth,  1280, 720, rs.format.z16, 30)

# 配置color流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
# config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
# config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
 
# streaming流开始
pipe_profile = pipeline.start(config) 

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
# 获取深度传感器的深度标尺（参见rs - align示例进行说明）
depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)      # ('Depth Scale is: ', 0.0010000000474974513)
 

# 创建对齐对象与color流对齐
# align_to 是计划对齐深度帧的流类型
align_to = rs.stream.color
# rs.align 执行深度帧与其他帧的对齐
align = rs.align(align_to)
 
 # Streaming循环
while True:
    ''' 
    获取图像帧与相机参数
    '''
    # 等待获取图像帧，获取颜色和深度的框架集
    frames = pipeline.wait_for_frames()         # frames.get_depth_frame（）是640x360深度图像
    # 获取对齐帧，将深度框与颜色框对齐
    aligned_frames = align.process(frames)
    # 获取对齐帧中的的depth帧
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame是640x480深度图像
    # 获取对齐帧中的的color帧
    aligned_color_frame = aligned_frames.get_color_frame()

    # 将images转为numpy arrays
    # RGB图
    img_color = np.asanyarray(aligned_color_frame.get_data())
    # 深度图（默认16位）
    img_depth = np.asanyarray(aligned_depth_frame.get_data())
 
    # Intrinsics & Extrinsics
    # 获取深度参数（像素坐标系转相机坐标系会用到）
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
    # 获取相机内参
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics
    # 获取两摄像头之间的外参
    depth_to_color_extrin = aligned_depth_frame.profile.get_extrinsics_to(aligned_color_frame.profile)

    # 设置测试随机点
    # Map depth to color
    depth_pixel = [320, 240]   # Random pixel
    x = depth_pixel[0]
    y = depth_pixel[1]


    ''' 
    方法一：获取三维坐标(rs2_deproject_pixel_to_point方法)
    '''
    # rs2_deproject_pixel_to_point方法，2d转3d，获得三维坐标
    # camera_coordinate = rs.rs2_deproject_pixel_to_point(intrin=depth_intrin, pixel=[x, y], depth=dis)
    # depth_intrin 从上一步获取
    # x 像素点的x
    # y 像素点的y
    # dis 上一步计算的真实距离（输入的dis与输出的距离是一样的，改变的只是x与y
    dis = aligned_depth_frame.get_distance(x, y)         # 深度单位是m
    print ('===============方法1：二维映射三维函数=============')
    print ('depth: ',dis)       # ('depth: ', 2.502000093460083)
    
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    print ('camera_coordinate: ',camera_coordinate)     # ('camera_coordinate: ', [-0.022640999406576157, -0.03151676058769226, 2.5230000019073486])

    color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, camera_coordinate)
    color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
    print ('color_point: ',color_point)     # ('color_point: ', [-0.022640999406576157, -0.03151676058769226, 2.5230000019073486])
    print ('color_pixel: ',color_pixel)     # ('color_pixel: ', [320.0, 240.0])
 

  

    ''' 
    方法二：获取三维坐标（点云的另一种计算方法）
    '''
    print ('===============方法2：点云=============')
    # pc = rs.pointcloud()
    # frames = pipeline.wait_for_frames()
    # depth = frames.get_depth_frame()
    # color = frames.get_color_frame()
    # img_color = np.asanyarray(color_frame.get_data())
    # img_depth = np.asanyarray(depth_frame.get_data())
    pc.map_to(aligned_color_frame)
    points = pc.calculate(aligned_depth_frame)
    vtx = np.asanyarray(points.get_vertices())
    tex = np.asanyarray(points.get_texture_coordinates())

    npy_vtx = np.zeros((len(vtx), 3), float)
    for i in range(len(vtx)):
        npy_vtx[i][0] = np.float(vtx[i][0])
        npy_vtx[i][1] = np.float(vtx[i][1])
        npy_vtx[i][2] = np.float(vtx[i][2])

    npy_tex = np.zeros((len(tex), 3), float)
    for i in range(len(tex)):
        npy_tex[i][0] = np.float(tex[i][0])
        npy_tex[i][1] = np.float(tex[i][1])

    print ('        ----------计算方法1：先转浮点，再i查找-----------')
    print('npy_vtx_shape: ', npy_vtx.shape)     # (307200, 3)
    print('npy_tex_shape:  ', npy_tex.shape)     # (307200, 3)

    i = y*640+x

    print('pointcloud_output_vtx: ', npy_vtx[i])     # array([-0.02245255, -0.03125443,  2.50200009])
    print('pointcloud_output_tex: ', npy_tex[i])     # array([ 0.5,  0.5,  0. ])

 
    
    ''' 
    方法三：获取三维坐标(点云方法)
    '''
    pc.map_to(aligned_color_frame)
    points = pc.calculate(aligned_depth_frame)
    
    vtx = np.asanyarray(points.get_vertices())

    print ('        ----------计算方法2：先i查找，再转浮点-----------')
    print ('vtx_before_reshape: ', vtx.shape)        # 307200
    i = y * 640 + x
    print ('test_output_point', [np.float(vtx[i][0]),np.float(vtx[i][1]),np.float(vtx[i][2])])      # ('test_output_point', [-0.022542288526892662, -0.031379349529743195, 2.51200008392334])


    print ('        ----------计算方法3：reshape后数组查找-----------')
    vtx = np.reshape(vtx,(480, 640, -1))   
    print ('vtx_after_reshape: ', vtx.shape)       # (480, 640, 1)

    # 注意顺序是 y，x；而不是 x，y
    # print ('output_point', vtx[y][x])       # ('output_point', array([(-0.022641, -0.03151676,  2.523)], dtype=[('f0', '<f4'), ('f1', '<f4'), ('f2', '<f4')]))
    print ('output_point', vtx[y][x][0])        # ('output_point', (-0.022641, -0.03151676,  2.523))

    tex = np.asanyarray(points.get_texture_coordinates())


    ''' 
    显示图像并显示三维坐标信息（以方法3结果为例）
    '''
    # 点的位置
    cv2.circle(img_color, (320,240), 8, [255,0,255], thickness=-1)
    # 深度从img_depth[x, y]中获得
    cv2.putText(img_color,"Dis:"+str(img_depth[320,240])+" m", (40,40), cv2.FONT_HERSHEY_SIMPLEX, 1.2,[255,0,255])
    cv2.putText(img_color,"X:"+str(np.float(vtx[y][x][0][0]))+" m", (80,80), cv2.FONT_HERSHEY_SIMPLEX, 1.2,[255,0,255])
    cv2.putText(img_color,"Y:"+str(np.float(vtx[y][x][0][1]))+" m", (80,120), cv2.FONT_HERSHEY_SIMPLEX, 1.2,[255,0,255])
    cv2.putText(img_color,"Z:"+str(np.float(vtx[y][x][0][2]))+" m", (80,160), cv2.FONT_HERSHEY_SIMPLEX, 1.2,[255,0,255])
    # 显示画面
    cv2.imshow('RealSence',img_color)
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break


pipeline.stop()
 

 

