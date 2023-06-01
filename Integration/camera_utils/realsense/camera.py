import pyrealsense2 as rs
import numpy as np
import time
import cv2

class Camera():
    def __init__(self, fps=30):
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            for s in device.sensors:
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    found_rgb = True
                    break
            assert found_rgb
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
            profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.pointcloud = rs.pointcloud()
        
        except Exception as e:
            print(e)
            exit(0)
               
    def get_frame(self, mode='original'):
        aligned_color_frame, aligned_depth_frame = False, False
        while not aligned_depth_frame or not aligned_color_frame:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

        self.pointcloud.map_to(aligned_color_frame)
        
        depth_image = np.array(aligned_depth_frame.get_data())
        color_image = np.array(aligned_color_frame.get_data())
        
        point_cloud = self.pointcloud.calculate(aligned_depth_frame)
        point_cloud = np.array(point_cloud.get_vertices())
        point_cloud = np.reshape(point_cloud, (480, 640))
        
        if mode == 'rotate':
            depth_image = np.flipud(np.fliplr(depth_image))
            color_image = np.flipud(np.fliplr(color_image))
            point_cloud = np.flipud(np.fliplr(point_cloud))
        
        return depth_image, color_image, point_cloud
    
    def close(self):
        self.pipeline.stop()


def streaming():
    camera = Camera(fps=6)
    while True:
        t1 = time.time()
        frame, cloud = camera.get_frame()
        cv2.imshow('RGB', frame)
        cv2.waitKey(1)
        t2 = time.time()
        print('fps = %1.1f' % (1 / (t2 - t1)))


def test_point_cloud():
    camera = Camera(fps=6)
    px, py = 320, 240
    i = 0 
    while True:
        frame, cloud = camera.get_frame(mode='rotate')
        frame = cv2.circle(frame, (px, py), 5, (0, 255, 0), -1)
        print('Cloud point', cloud[py, px])
        cv2.imshow('RGB', frame)
        cv2.waitKey(100)
        i += 1
        if i == 50:
            i = 0
            px = np.random.randint(0, 640)
            py = np.random.randint(0, 480)



if __name__ == "__main__":
##    streaming()
    test_point_cloud()
