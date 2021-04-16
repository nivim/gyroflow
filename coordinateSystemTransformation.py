import numpy as np
from scipy.spatial.transform import Rotation as R

'''
coordinateSystemTransformation
Converts camera IMU data to standard right-handed Cartesian coordinate system ('right handed' XYZ coordinate system) as a point of reference to all convertions.
It supports mostly used cameras and can add on the fly.
Transforms camera rotations (Tilt and Rotations) to align the camera to the IMU
gyroFlow coordinate system (In XYZ terms) is -Z, -X, Y
'''

class coordinateSystemTransformation:
    def __init__(self, input_coordinate_system='gyroflow', rotation=None, gyro_data_input=None, acc_data_input=None):
        # Orientation transformation TO!! XYZ
        self.orientation_libary = {
            'hero5' : np.array([180,-90,0]),
            'hero6' : np.array([-90,0,90]),
            'hero7' : np.array([-90,0,90]),
            'hero8' : np.array([0,90,0]),
            'oner' : np.array([0,0,0]),
            'smo4k' : np.array([0,180,0]),
            'go' : np.array([-90,0,0]),
            # Assuming blackbox is already converted to gyroflow format
            'blackbox' : np.array([-90,0,90]),
            'gyroflow' : np.array([-90,0,90])
        }

        self.input_orientation = [0,0,0]
        self.output_orientation = self.orientation_libary.get('gyroflow') *-1
        self.is_data = False
        self.gyro_data_xyz = None
        self.acc_data_xyz = None

        if self.orientation_libary.get(input_coordinate_system) is None:
            # If input string is invalid, assumes gyroflow coordinate system
            self.input_orientation = self.orientation_libary.get('gyroflow')
        else:
            self.input_orientation = self.orientation_libary.get(input_coordinate_system)
        
        if rotation is not None:
            self.input_orientation = self.input_orientation + np.array([rotation])

        if gyro_data_input is not None:
            self.input_data(gyro_data_input, acc_data=acc_data_input)

    def __transform_system(self, data, transform):
        '''
        XYZ traformation explaination (Right Handed):
        X Positive turns clockwise (Right)
        Y Positive turns forward
        Z Positive turns left
        '''
        r = R.from_euler('xyz', transform, degrees=True)
        return r.apply(data)

    def __dataset_transform(self, dataset, transform):
        transformed_dataset = np.empty(dataset.shape)
        transformed_dataset[:,-3:] = self.__transform_system(dataset[:,-3:], transform)
        if dataset[:,-4:-3].shape[1] > 0:
            transformed_dataset[:,0] = dataset[:,0]
        return transformed_dataset

    def __camera_transform(self, gyro_in, transform, acc_in=None):
        gyro_out = self.__dataset_transform(gyro_in, self.input_orientation)
        if acc_in is not None:
            acc_out = self.__dataset_transform(acc_in, transform)
        else:
            acc_out = None
        return gyro_out, acc_out

    def input_data(self, gyro_data_input, acc_data_input=None):
        
        self.gyro_data_xyz, self.acc_data_xyz = self.__camera_transform(gyro_data_input, self.input_orientation, acc_in=acc_data_input)
        self.is_data = True
        self.__as_gyroflow()

    def __as_gyroflow(self):
        self.gyro_data_gyroflow, self.acc_data_gyroflow = self.__camera_transform(self.gyro_data_xyz, self.output_orientation, acc_in=self.acc_data_xyz)

    def gyroflow(self):
        if not self.is_data:
            return False
        if self.acc_data_xyz is not None:
            return self.gyro_data_gyroflow ,self.acc_data_gyroflow
        else:
            return self.gyro_data_gyroflow

    def export_as(output_coordinate_system):
        if not self.is_data:
            return False
        return self.__camera_transform(self.gyro_data_xyz, self.orientation_libary.get(output_coordinate_system), acc_in=self.acc_data_xyz)
        
    
if __name__ == "__main__":
    transform = coordinateSystemTransformation(input_coordinate_system='smo4k')
    transform.input_data(gyro_data_input=np.array([[0,-1,-2,-3], [1,4,5,6]]))
    print(transform.gyro_data_xyz)
    print(transform.gyroflow())