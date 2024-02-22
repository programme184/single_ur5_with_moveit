#!/usr/bin/env python
import cv2.aruco as aruco
import numpy as np
import cv2
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import curve_fit
from scipy.interpolate import splrep, splev
from realsense_img import ros_camera
from aruco_detec_pose_estimate import aruco_pose
from Translation import Transformation
# from UR_env import Ur5_env


markers = aruco_pose()
rl_cam = ros_camera(node_name='test')


class rope_model:
    
    def data_extract(self, image, aruco_dict, end_position, end_orien):
        
        markers.PoseEstimate(image, aruco_dict, aruco_size=0.007)
        # ids, tvec = aruco_pose.ids, aruco_pose.tvec
        trans_dic = markers.to_wordcoord(end_position, end_orien)
        
        pos_list = []
        for id in trans_dic.keys():
            pos = trans_dic[id][0]
            orien = trans_dic[id][1]
            pos_world, _ = Transformation.world_base_trans(pos, orien, 0)
            pos_list.append(pos_world)
        
        return pos_list    
    
    @staticmethod
    def plot_data(data):
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract x, y, z values for each pose
        x, y, z = zip(*data)
        # Plot the scatter points
        ax.scatter(x, y, z, c='blue', marker='o')
        # Set axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_title('Scatter Plot of Aruco Poses')

        # plt.show()


    def data_interpolation(self, data):
        x, y, z = zip(*data)
        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        # Ensure x values are in ascending order
        sorted_indices = np.argsort(x)
        x = np.array(x)[sorted_indices]
        y = np.array(y)[sorted_indices]
        z = np.array(z)[sorted_indices]
        
        # Remove duplicate x values
        unique_indices = np.unique(x, return_index=True)[1]
        x = x[unique_indices]
        y = y[unique_indices]
        z = z[unique_indices]
        
        # Use cubic spline interpolation for each dimension
        try:
            tck_x = splrep(x, z, s=0)
            tck_y = splrep(x, y, s=0)
            # Evaluate the spline over a finer range
            x_fine = np.linspace(min(x), max(x), 100)
            y_fine = splev(x_fine, tck_y)
            z_fine = splev(x_fine, tck_x)

            # Plot the data points
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.scatter(x, y, z, c='blue', marker='o', label='Data')

            # Plot the fitted spline
            ax.plot(x_fine, y_fine, z_fine, color='red', label='Fitted Spline')

            # Set axis labels
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            # Set plot title
            ax.set_title('Fitted 3D Spline to Data')
            # Show the plot
            # plt.show()
            

        except ValueError as e:
            print(f"Spline interpolation failed: {e}")
        
        
    def data_fitting(self, data):
        # Perform curve fitting 
        x, y, z = zip(*data)
        x = np.array(x)
        y = np.array(y)
        z = np.array(z)
        # print('data x', type(x), x)
        # print('(x, y)', (x, y), type((x, y)))
        popt, pcov = curve_fit(self.func, (x, y), z) 
        # Print optimized parameters 
        # print(popt)
        
                # Extract the fitted parameters
        a, b, c, d, e, f = popt

        # Limit the range of values for plotting
        x_range = np.linspace(min(x), max(x), 100)
        y_range = np.linspace(min(y), max(y), 100)
        xx, yy = np.meshgrid(x_range, y_range)

        # Create a meshgrid for plotting the fitted surface
        fitted_surface = self.func((xx, yy), a, b, c, d, e, f)
        # Create a meshgrid for plotting the fitted surface
        # xx, yy = np.meshgrid(np.linspace(min(x), max(x), 100), np.linspace(min(y), max(y), 100))
        # # fitted_surface = self.func((xx, yy), a, b, c, d, e, f)
        # fitted_surface = self.curve2d((xx, yy), a, b, c, d)

        # Plot the data points
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d') 
        # ax = fig.add_subplot(111, projection='3d')
        ax.scatter(x, y, z, c='blue', marker='o', label='Data')

        # Plot the fitted surface
        ax.plot_surface(xx, yy, fitted_surface, alpha=0.5, color='red', label='Fitted Surface')
        # Plot the fitted helix
        # ax.plot(xx, yy, fitted_surface, color='red', label='Fitted Helix')
    
        ax.set_xlabel('X') 
        ax.set_ylabel('Y') 
        ax.set_zlabel('Z') 
        # plt.show()
    
    def func(self, xy, a, b, c, d, e, f): 
        x, y =  xy
        return a + b*x + c*y + d*x**2 + e*y**2 + f*x*y
    
    
    # Define a 3D curve function
    def curve2d(slef, xy, a, b, c, d):
        x, y = xy
        return a + b*x + c*y + d*x*y

        # Define a 3D polynomial function
    def poly3d(self, xy, a, b, c, d, e, f, g, h, i):
        x, y = xy
        return a + b*x + c*y + d*x**2 + e*x*y + f*y**2 + g*x**3 + h*x**2*y + i*y**3




if __name__=="__main__":
    # === fixed end effector for debug
    # joints: [2.7833564281463623, -2.8213680426227015, -0.5524399916278284, -2.046360794697897, -0.761390511189596, 1.8266359567642212]
    position = [0.7760001412233319, -0.5098467808438778, 0.19340444926009234]
    orientation = [0.19203986270253257, -0.4483419875220519, 0.8541455076601253, 0.18040400522714453]
    
    # rospy.init_node("image", anonymous=False)
    
    rope = rope_model()
    # robot = Ur5_env()
    # position, orientation = robot.info_get()
    image = rl_cam.get_image()
    # aruco_dict4 = aruco.Dictionary_get(aruco.DICT_4X4_250)
    aruco_dict5 = aruco.Dictionary_get(aruco.DICT_5X5_250)

    pose_data = rope.data_extract(image, aruco_dict5, position, orientation)
    print('pose', pose_data)
    rope_model.plot_data(pose_data)
    rope.data_fitting(pose_data)
    rope.data_interpolation(pose_data)
    # Show the plot
    plt.show()
    # Destroy all OpenCV windows when finished
    # cv2.destroyAllWindows()