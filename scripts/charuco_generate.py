import numpy as np
import cv2, PIL, os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd


# # workdir = "data/"

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

fig = plt.figure(figsize=(210 / 25.4, 297 / 25.4), dpi=300)  # Size in mm
nx = 2
ny = 4
for i in range(1, nx*ny+1):
    ax = fig.add_subplot(ny,nx, i)
    img = aruco.drawMarker(aruco_dict,i+70, 20)
    plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")

plt.savefig("markers.pdf")
# plt.show()
#plt.close()


# board = aruco.CharucoBoard_create(6, 8, 0.03, 0.022, aruco_dict)
# imboard = board.draw((210, 297))
# # fig = plt.figure()
# ax = fig.add_subplot(1,2,1)
# plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
# ax.axis("off")
# plt.savefig("chessboard.pdf")
plt.show()




# import cv2
# from cv2 import aruco
# from PIL import Image
# import os

# def generate_aruco_board(marker_id, board_size, square_length, marker_length):
#     aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
#     board = aruco.CharucoBoard_create(
#         squaresX=board_size[0],
#         squaresY=board_size[1],
#         squareLength=square_length,
#         markerLength=marker_length,
#         dictionary=aruco_dict
#     )
#     image = board.draw((210 * 4, 297 * 4))
#     cv2.imwrite(f"aruco_board_{marker_id}.png", image)
#     return image

    
# def save_to_pdf(images, pdf_filename):
#     pil_images = []
#     for image in images:
#         pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
#         pil_images.append(pil_image)

#     first_image = pil_images[0]
#     pdf_width, pdf_height = first_image.size
#     pdf_height_mm = 297
#     scale_factor = pdf_height_mm / pdf_height

#     pdf_images = [image.resize((int(pdf_width * scale_factor), int(pdf_height * scale_factor)), Image.ANTIALIAS) for image in pil_images]
#     pdf_filename_with_extension = os.path.splitext(pdf_filename)[0] + ".pdf"
#     pdf_images[0].save(pdf_filename_with_extension, "PDF", save_all=True, append_images=pdf_images[1:])
    
    
# if __name__ == '__main__':
#     # Generate ArUco boards
#     marker_id_1 = 1
#     board_size_1 = (5, 2)
#     square_length_1 = 30
#     marker_length_1 = 20

#     marker_id_2 = 2
#     board_size_2 = (4, 2)
#     square_length_2 = 40
#     marker_length_2 = 30

#     # image_path_1 = generate_aruco_board(marker_id_1, board_size_1, square_length_1, marker_length_1)
#     # image_path_2 = generate_aruco_board(marker_id_2, board_size_2, square_length_2, marker_length_2)

#     # # Save to PDF
#     # save_to_pdf([image_path_1, image_path_2], "aruco_boards.pdf")
    
    
#     # Generate ArUco boards
#     image_1 = generate_aruco_board(marker_id_1, board_size_1, square_length_1, marker_length_1)
#     image_2 = generate_aruco_board(marker_id_2, board_size_2, square_length_2, marker_length_2)

#     # Save to PDF
#     save_to_pdf([image_1, image_2], "aruco_boards.pdf")