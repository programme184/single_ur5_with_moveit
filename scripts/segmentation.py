import cv2

import matplotlib.pyplot as plt
import numpy as np
from realsense_img import ros_camera
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors


class rope_segmentation:
    def hsv_img(self):
        rl_cam = ros_camera(node_name='func')
        image = rl_cam.get_image()

        contrast = 1.2 # Contrast control ( 0 to 127)
        brightness = 0. # Brightness control (0-100)

        out = cv2.addWeighted( image, contrast, image, 0, brightness)
        whale_hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(whale_hsv)

        pixel_colors = whale_hsv.reshape((np.shape(whale_hsv)[0]*np.shape(whale_hsv)[1], 3))
        norm = colors.Normalize(vmin=-1.,vmax=1.)
        norm.autoscale(pixel_colors)
        pixel_colors = norm(pixel_colors).tolist()

        lower_blue = np.array([99, 40, 30], dtype=np.uint8)
        upper_blue = np.array([120, 255, 170], dtype=np.uint8)

        mask = cv2.inRange(whale_hsv, lower_blue, upper_blue)
        result = cv2.bitwise_and(out, out, mask=mask)
        
        return whale_hsv, result

    def contour_detec(self):
        _, result = self.hsv_img()
        gray_image = cv2.cvtColor(result, cv2.COLOR_RGB2GRAY)
        (thresh, binary_image) = cv2.threshold(gray_image, 1, 255, cv2.THRESH_BINARY)

        # Convert the result to a binary image
        _, binary_result = cv2.threshold(cv2.cvtColor(result, cv2.COLOR_RGB2GRAY), 1, 255, cv2.THRESH_BINARY)
        binary_result[binary_result > 0] = 255
        kernel = np.array([[0, 1, 0],
                   [1, 1, 1],
                   [0, 1, 0]], dtype=np.uint8)
        # Apply dilation
        dilated_image = cv2.dilate(binary_image, kernel, iterations=1)
        plt.imshow(dilated_image, cmap="gray")
        # Apply erosion
        eroded_image = cv2.erode(dilated_image, kernel, iterations=1)
        # Image preprocessing
        blurred = cv2.GaussianBlur(eroded_image, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)


if __name__ == "__main__":
    seg = rope_segmentation()
    seg.contour_detec()
    # plt.subplot(1, 2, 1)
    # plt.imshow(mask, cmap="gray")
    # plt.subplot(1, 2, 2)
    # plt.imshow(result)
    # plt.show()
# h, s, v = cv2.split(whale_hsv)

# # plt.show()
# lower_black = np.array([25, 0, 0], dtype=np.uint8)
# upper_black = np.array([100, 140, 70], dtype=np.uint8)


# mask = cv2.inRange(hsv_nemo, lower_black, upper_black)
# result = cv2.bitwise_and(nemo, nemo, mask=mask)
