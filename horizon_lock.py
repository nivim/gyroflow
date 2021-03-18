import numpy as np
from ahrs.filters import Madgwick, AngularRate
from ahrs import QuaternionArray
import cv2
import math

class horizon_locker:
    # Filter data based on presented data (with or without accelerometer data)
    def __init__(self, timestamps , gyro_data,data_frequency, acc_data=None ):
        if acc_data is not None:
            self.filtered_imu = Madgwick(gyr=gyro_data, acc=acc_data, frequency=float(data_frequency)) 
        else:
            self.filtered_imu = AngularRate(gyr=gyro_data, frequency=float(data_frequency))
        self.filtered_imu_in_angles = QuaternionArray(self.filtered_imu.Q).to_angles()
        self.roll_in_rads = self.filtered_imu_in_angles[:,0]
        self.timestamps = timestamps
        self.pointer_position = 0

# finds the element closest to the value - In this case the time closest to the frame time
    def __find_nearest_position(self, array, value):
        array = np.asarray(array)
        return (np.abs(array - value)).argmin()

    def get_horizon_deg_angle_by_frame(self, time_sec):
        self.pointer_position = self.__find_nearest_position(self.timestamps , time_sec)
        # return the angle in degrees
        return self.roll_in_rads[self.pointer_position] * 180/math.pi

    def rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def lock_horizon(self, time_sec, frame):
        angle = self.get_horizon_deg_angle_by_frame(time_sec)*-1
        return self.rotate_image(frame, angle)
