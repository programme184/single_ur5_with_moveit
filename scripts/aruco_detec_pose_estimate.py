
# from jaka_move import ManiControl
import cv2.aruco as aruco
import numpy as np
from Translation import Transformation
import time
import cv2

# D455
Dist = np.array([-0.05859437212347984, 0.07361707091331482, 0.00019908350077457726, -0.0006242774543352425, -0.02336048148572445])  # system given


mtx=np.array([[633.0994262695312, 0.0, 641.6953125],
 [  0.       ,  632.49951171875, 365.11553955078125],
 [  0.,           0.,           1.        ]])

# D: [0.14574457705020905, -0.49072766304016113, 0.0002240741887362674, -0.00014576673856936395, 0.4482966661453247]
# K: [901.964599609375, 0.0, 652.5621337890625, 0.0, 902.0592651367188, 366.7032165527344, 0.0, 0.0, 1.0]

# P: [[901.964599609375, 0.0, 652.5621337890625], [0.0, 902.0592651367188, 366.7032165527344], , 0.0, 1.0, 0.0]

class aruco_pose:
    parameters =  aruco.DetectorParameters_create()
    
    ids = None
    rvec, tvec = None, None
    
    def PoseEstimate(self, image, aruco_dict, aruco_size=0.05, draw=False, axis_size=0.03):
        parameters =  aruco.DetectorParameters_create()
        #使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict,parameters=parameters)
        # time.sleep(3)
        # print('image:', image)
        if draw is False:
            while True: 
                corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict,parameters=self.parameters)
                if ids is not None:
                    # print('ID', ids)
                    break
            # for i in range(0, 3):
        else:
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict,parameters=parameters)
                # time.sleep(2)
        if ids is not None:

            self.ids = ids
            
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, aruco_size, mtx, Dist)       #output shape:(4,1,3)
            
            self.rvec = rvec
            self.tvec = tvec
            # print('tvec:\n', tvec)
            if draw is True:
                for i in range(rvec.shape[0]):
                    cv2.drawFrameAxes(image, mtx, Dist, rvec[i, :, :], tvec[i, :, :], axis_size)
                    aruco.drawDetectedMarkers(image, corners)
                ###### DRAW ID #####
                cv2.putText(image, "Id: " + str(ids), (0,64), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),2,cv2.LINE_AA)
            else:
                print('Detected aruco:', ids.reshape(1, -1))
                
        else:
            if draw is False:
                print('No aruco ID')
                raise AssertionError

    def to_wordcoord(self, end_pos, end_ori):
        # when rvec and tvec are not empty
        trans = Transformation()
        
        translation_dic = {}
        for i in range(self.rvec.shape[0]):
            ids, ti = self.ids, self.tvec[i, :, :]
            # print('ti', ti)
            rotatei = trans.RotMatrix(self.rvec[i, :, :], rodrigues=True)
            
            tr, quat_t = trans.express_transform(ti, rotatei, end_pos, end_ori)

            id = ids[i][0]
            translation_dic[id]=[tr, quat_t]

        
        return translation_dic
    
    
    # def to_word_pose(self, end_pos, end_ori):
    #     # when rvec and tvec are not empty
    #     trans = Transformation()
        
    #     translation_dic = {}
    #     for i in range(self.rvec.shape[0]):
    #         ids, ti = self.ids, self.tvec[i, :, :]
    #         # print('ti', ti)
    #         rotatei = trans.RotMatrix(self.rvec[i, :, :], rodrigues=True)
            
    #         tr, quat_t = trans.express_transform(ti, rotatei, end_pos, end_ori)

    #         id = ids[i][0]
    #         translation_dic[id]=[tr, quat_t]

        
    #     return translation_dic
    # def pose_world(self, image):
    #     robot_control = ManiControl()