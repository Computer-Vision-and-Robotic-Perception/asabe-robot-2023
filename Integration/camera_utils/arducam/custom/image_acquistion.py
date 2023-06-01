from __future__ import print_function
import arducam_mipicamera as arducam
import v4l2
import time
from datetime import datetime
import os

#makes input function compatible for Python2 and Python3
try:
    input = raw_input
except NameError:
    pass


#-------Camera settings-----------
red_gain = 100
blue_gain = 100
exposure = 812
#exposure = 1762
#exposure = 11312
resolution = [4192, 3120]

#-----File Saving Variables------
#Parent Folder
parent_path = r'/home/pi/Desktop/MIPI_images/'
#: Date format.
date_format = '%m-%d-%Y'
#: Time format.
time_format = '%H:%M:%S'
#image number
sample_number = 1
#Dataset number
dataset_number = None

def find_dataset_number(path):
    """
    This method finds and assigns the next available dataset number in 
    the given location (path).

    :param  str path: path where to find the next available dataset number.
    """
    #start at 1
    number = 1
    directory = path + '/DATASET_' + str(number)
    #loop until the the available directory has been found
    while os.path.isdir(directory):
        #increment  number
        number += 1
        directory = path + '/DATASET_' + str(number)
    return number

def get_image_saving_path():
    global dataset_number
    #get current time and date
    now = datetime.now()
    #format date
    date_ = datetime.strftime(now,date_format)
    #time_ = datetime.strftime(now,time_format)

    #path to parent directories: parent directory name + date
    saving_path = parent_path + '/' + date_ + "/"

    #if parent directory doesn't exits
    if not os.path.isdir(saving_path):
        os.makedirs(saving_path) #make directory

    #check if data set number has been set
    if dataset_number == None:
        #find the next avaible dataset number
        dataset_number = find_dataset_number(saving_path)

    #append 'DATASET_#' folder to saving path
    saving_path = saving_path + '/DATASET_' + str(dataset_number) + '/'

    #if parent directory doesn't exits
    if not os.path.isdir(saving_path):
        os.makedirs(saving_path) #make directory

    return saving_path

def capture():
    global sample_number
    frame1 = camera.capture(encoding = 'jpeg')
    saving_path = get_image_saving_path()
    file_name = saving_path + '/' + '{}.jpg'.format(sample_number)
    frame1.as_array.tofile(file_name)
    # Release memory
    del frame1
    #camera.stop_preview()
    print('Image {} saved'.format(sample_number))
    sample_number += 1

try:
    camera = arducam.mipi_camera()
    print("Open camera...")
    camera.init_camera()
    fmt = camera.set_resolution(resolution[0], resolution[1])
    print("Current resolution is {}".format(fmt))
    print("Disable Auto White balance...")
    camera.software_auto_white_balance(enable = False)
    time.sleep(1)
    
    #print("Enable Auto Exposure...")
    #camera.software_auto_exposure(enable = True)
    #time.sleep(1)
    
    print("Disable Auto Exposure...")
    camera.software_auto_exposure(enable = False)
    time.sleep(1)
    print("Setting the exposure to " + str(exposure) + " ms")
    camera.set_control(v4l2.V4L2_CID_EXPOSURE, exposure)
    time.sleep(1)
    
    #print("Setting red gain to {}, blue gain to {}".format(red_gain,blue_gain))
    #camera.manual_set_awb_compensation(red_gain,blue_gain)
    #time.sleep(1)
    
    camera.start_preview(fullscreen= False, window=(10,0,1280,720))
    while True:
        user_input = input("\nPress enter to \'T\' to start preview or \'Q\' to quit: ")
        if user_input == 't' or user_input == 'T':
            capture()
        if user_input == 'q' or user_input == 'Q':
            break


        time.sleep(0.5)

    camera.stop_preview()
    print('\nClosing camera')
    camera.close_camera()

except Exception as e:
    print(e)
    print('\nclosing camera')
    camera.close_camera()


