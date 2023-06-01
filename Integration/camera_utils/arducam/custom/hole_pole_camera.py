import arducam_mipicamera as arducam
import time
import v4l2

class Camera():
    def __init__(self):
        try:
            self.camera = arducam.mipi_camera()
            print("Open camera...")
            self.camera.init_camera()
            self.camera.set_mode(0)
            self.fmt = self.camera.get_format()
            self.fmt = (self.fmt["width"], self.fmt["height"])
            print("Current resolution is {}".format(self.fmt))
            self.camera.software_auto_white_balance(enable=False)
            time.sleep(1)
            self.camera.software_auto_exposure(enable=False)
            time.sleep(1)
            self.camera.set_control(v4l2.V4L2_CID_EXPOSURE, 5000)
            time.sleep(1)
            self.camera.set_control(v4l2.V4L2_CID_FOCUS_ABSOLUTE, 550)
            time.sleep(1)
            print('Setup done!')
         
        except Exception as e:
            print(e)
            
            
    def get_frame(self, kind='raw'):
        if kind == 'raw':
            data = self.camera.capture(encoding = 'raw')
            frame = arducam.unpack_raw10_to_raw8(data.buffer_ptr, self.fmt[0], self.fmt[1])
            gray = frame.as_array.reshape((self.fmt[1], self.fmt[0]))
        else:
            data = self.camera.capture(encoding = 'raw')
            frame = arducam.unpack_raw10_to_raw8(data.buffer_ptr, self.fmt[0], self.fmt[1])
            gray = frame.as_array.reshape((self.fmt[1], self.fmt[0]))
        return gray.copy()
    
    def close(self):
        self.camera.close_camera()


if __name__ == "__main__":
    camera = Camera()
    while True:
        t1 = time.time()
        frame = camera.get_frame()
        t2 = time.time()
        print('fps = %3.4f' % (1 / (t2 - t1)))
