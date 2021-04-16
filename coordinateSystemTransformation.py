import numpy as np
from scipy.spatial.transform import Rotation as R

class coordinateSystemTransformation:
    def __init__(self, input_coordinate_system='gyroflow', rotation=None, gyro_data_input=None, acc_data_input=None):
        # Orientation transformation TO!! XYZ
        self.orientation_libary = {
            'hero5' : np.array([0,0,0]),
            'hero6' : np.array([0,0,0]),
            'hero7' : np.array([0,0,0]),
            'hero8' : np.array([0,0,0]),
            'oner' : np.array([0,0,0]),
            'smo4k' : np.array([0,180,0]),
            'go' : np.array([0,0,0]),
            'blackbox' : np.array([0,0,0]),
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
        
        # if input_coordinate_system=='hero5':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='hero6':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='hero7':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='hero8':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='hero9':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='oner':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='smo4k':
        #     self.input_orientation = np.array([0,180,0])
        # elif input_coordinate_system=='go':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='go2':
        #     self.input_orientation = np.array([0,0,0])
        # elif input_coordinate_system=='gyroflow':
        #     self.input_orientation = np.array([-90,0,90])
        # else:
        #     # Assumes GyroFlow
        #     self.input_orientation = np.array([-90,0,90])
        
        if rotation is not None:
            self.input_orientation = self.input_orientation + np.array([rotation])

        if gyro_data_input is not None:
            self.input_data(gyro_data_input, acc_data=acc_data_input)

    def __transform_system(self, data, transform):
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
        self.gyro_data_xyz = self.__dataset_transform(gyro_data_input, self.input_orientation)
        if acc_data_input is not None:
            self.acc_data_xyz = self.__dataset_transform(acc_data_input, self.input_orientation)
        self.is_data = True
        self.__to_gyroflow()

    def __to_gyroflow(self):
        self.gyro_data_gyroflow = self.__dataset_transform(self.gyro_data_xyz, self.output_orientation)
        if self.acc_data_xyz is not None:
            self.acc_data_gyroflow = self.__dataset_transform(self.acc_data_xyz, self.output_orientation)


    # def __convert_to_XYZ(self, data):
    #     r = R.from_euler('xyz', self.input_orientation, degrees=True)
    #     return r.apply(data)

    # def __convert_to_gyroflow(self, data):
    #     r = R.from_euler('xyz', [90,0,-90], degrees=True)
    #     return r.apply(data)

    # def input_data(self, gyro_data, acc_data=None):
    #     self.gyro_data_xyz = np.empty(gyro_data.shape)
    #     self.gyro_data_xyz[:,0] = gyro_data[:,0]
    #     self.gyro_data_xyz[:,1:4] = self.__convert_to_XYZ(gyro_data[:,1:4])

    #     if acc_data is not None:
    #         self.acc_data_xyz = np.empty(acc_data.shape)
    #         self.acc_data_xyz[:,0] = acc_data[:,0]
    #         self.acc_data_xyz[:,1:4] = self.__convert_to_XYZ(acc_data[:,1:4])
    #     self.is_data = True
    #     self.__to_gyroflow()

    # def __to_gyroflow(self):
    #     self.gyro_data_gyroflow = np.empty(self.gyro_data_xyz.shape)
    #     self.gyro_data_gyroflow[:,0] = self.gyro_data_xyz[:,0]
    #     self.gyro_data_gyroflow[:,1:4] = self.__convert_to_gyroflow(self.gyro_data_xyz[:,1:4])

    #     if self.acc_data_xyz is not None:
    #         self.acc_data_gyroflow = np.empty(self.acc_data_xyz.shape)
    #         self.acc_data_gyroflow[:,0] = self.acc_data_xyz[:,0]
    #         self.acc_data_gyroflow[:,1:4] = self.__convert_to_gyroflow(self.acc_data_xyz[:,1:4])


    def gyroflow(self):
        if not self.is_data:
            return False
        if self.acc_data_xyz is not None:
            return self.gyro_data_gyroflow ,self.acc_data_gyroflow
        else:
            return self.gyro_data_gyroflow

    def export_as(output_coordinate_system)
        if not self.is_data:
            return False
        return self.__camera_transform(self.gyro_data_xyz, self.orientation_libary.get(output_coordinate_system), acc_in=self.acc_data_xyz)
        
    
if __name__ == "__main__":
    transform = coordinateSystemTransformation(input_coordinate_system='smo4k')
    gyro_test = [ 6.00000000e+00, -5.32632228e-03,  3.19579337e-03, -1.70442313e-02]
    transform.output_orientation = transform.orientation_libary.get('gyroflow')*-1
    transform.input_data(gyro_data_input=np.array([[0,-1,-2,-3], [1,4,5,6]]))
    print(transform.gyro_data_xyz)
    print(transform.gyroflow())