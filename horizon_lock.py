import numpy as np
from ahrs.filters import Madgwick, AngularRate, AQUA, Complementary, Mahony
from ahrs import QuaternionArray
import cv2
import math
from scipy.stats import iqr

class horizon_locker:
    # Filter data based on presented data (with or without accelerometer data)
    def __init__(self, timestamps , gyro_data, time_delta, acc_data=None ,data_frequency=None ,limit_acc_range_percentiles=None, added_rotate_angle=0):
        # Limit acc data (prevent outliers)
        if limit_acc_range_percentiles is not None:
            acc_data=self.__limit_range(acc_data, range=(limit_acc_range_percentiles, 100-limit_acc_range_percentiles))
        # Calculate data frequency
        if data_frequency is None:
            data_frequency=1/np.mean(np.diff(timestamps))

        if acc_data is not None:
            self.filtered_imu = Madgwick(gyr=gyro_data, acc=acc_data, frequency=float(data_frequency)) 
        else:
            self.filtered_imu = AngularRate(gyr=gyro_data, frequency=float(data_frequency))
        self.filtered_imu_in_angles = QuaternionArray(self.filtered_imu.Q).to_angles()
        self.roll_in_rads = self.filtered_imu_in_angles[:,0]
        self.timestamps = timestamps
        self.pointer_position = 0
        self.time_delta = time_delta
        self.added_rotate_angle = added_rotate_angle

# finds the element closest to the value - In this case the time closest to the frame time
    def __find_nearest_position(self, array, value):
        array = np.asarray(array)
        return (np.abs(array - value)).argmin()

    def get_horizon_deg_angle_by_time(self, time_sec, time_delta):
        self.pointer_position = self.__find_nearest_position(self.timestamps , time_sec + time_delta)
        # return the angle in degrees
        return self.roll_in_rads[self.pointer_position] * 180/math.pi

    def __rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def lock_horizon(self, time_sec, frame):
        angle = self.get_horizon_deg_angle_by_time(time_sec, self.time_delta)*-1 + self.added_rotate_angle
        return self.__rotate_image(frame, angle)

    def __limit_range(self, data, range=(5,95)):
        num_rows, num_cols = data.shape
        i_list = [0,1,2]
        for i in i_list:
            iqr_val = iqr(data[:,i], rng=range)
            avg = np.average(data[:,i])
            data[:,i] = np.clip(data[:,i],avg-iqr_val,avg+iqr_val)
        return data

