import transforms3d as tfs
import numpy as np
import cv2

class Transformation:
  
  # 0.008755484	-0.056858451	0.039588841	-0.008405747	-0.023210214	0.001653124	0.999693827

  # cam_pos =[-0.038007,   0.0431236,  0.0367012]
  # cam_ori = [0.014383107367188808, -6.700755339672515e-05, 0.3911408087157479, 0.9202184466145209]#[0.01414568946395788, -0.00257750271097712, 0.39945193875924767, 0.9166413718411591] # xyzw[0, 0, 0,1]#[-0.9200639780864388, 0.3916255267279043, -0.006474514547970843, 0.008354861112946355]
  H_h2c = None
  
  def __init__(self, cam_pos=None, cam_ori=None):
    
    if cam_pos is None or cam_ori is None:
      # D455 in ur5
      self.cam_pos = [0.028010119, 0.064888998, 0.012021397]
      self.cam_ori =[-0.002033309, -0.003367025, 0.701532085, 0.712627015]#[0.01414568946395788, -0.00257750271097712, 0.39945193875924767, 0.9166413718411591] # xyzw[0, 0, 0,1]#[-0.9200639780864388, 0.3916255267279043, -0.006474514547970843, 0.008354861112946355]f
      # 0.024148504	0.058214922	0.01047461	-0.00167642	0.00327142	0.70113753	0.713015907
# self.cam_pos = [0.024148504, 0.058214922, 0.01047461]
#       self.cam_ori =[-0.00167642, 0.00327142, 0.70113753, 0.713015907]
    else:
      self.cam_pos = cam_pos
      self.cam_ori = cam_ori
  
  def RotMatrix(self, vector, rodrigues=False):                               # input vector should be array
    if rodrigues is False:
        quat = vector
        Rot = tfs.quaternions.quat2mat((quat[3], quat[0], quat[1], quat[2])) # quat:xyzw but the quat2mat input should be wxyz
    else:
        Rot = cv2.Rodrigues(vector)[0]
    
    return Rot
  
  def express_transform(self, T_obj, R_obj, end_pos, end_ori, item = None):
    # use 4x4 translation matrix T to realize
    
    Ro = self.RotMatrix(np.array(end_ori))
    To = np.array(end_pos)
    # print('end_pos:', end_pos)
    # print('end_ori:', end_ori)
    # T2 = np.array(obj_pos)
    
    # R2 = self.RotMatrix(np.array(obj_ori))

    R1 = self.RotMatrix(self.cam_ori)
    T1 = np.array(self.cam_pos)
    
    hand_camera_tr = T1
    hand_camera_rot = R1
    hand_world_tr = To
    hand_world_rot = Ro
    marker_camera_tr = T_obj
    marker_camera_rot = R_obj

    H_h2c = tfs.affines.compose(np.squeeze(hand_camera_tr), hand_camera_rot, [1, 1, 1])                 # hand to camera
    
    # base_link->end_link
    H_b2h = tfs.affines.compose(np.squeeze(hand_world_tr), hand_world_rot, [1, 1, 1])                   # get a translation matrix, like [r, t; 000 1], 4x4, base to hand
    # marker->camera  or camera->marker
    H_c2marker = tfs.affines.compose(np.squeeze(marker_camera_tr), marker_camera_rot, [1, 1, 1])       # camera to marker
    # base_link->end_link->marker   or base_link->end_link->camera 
    H_b2c = np.dot(H_b2h,H_h2c)                                                                        # base to camera 右乘<-运动坐标变换
    self.H_h2c = H_h2c
    
    
    if item is None or item ==3:                                            # calculate the world pose of a detected object
      # base_link->end_link->marker->camera  or base_link->end_link->camera->marker
      H_b2marker = np.dot(H_b2c,H_c2marker)                                 # base to marker
      tr = H_b2marker[0:3,3:4].T[0]

      quat = tfs.quaternions.mat2quat(H_b2marker[0:3,0:3])                  # wxyz form

      
    elif item == 2:                                                         # calculate the world pose of the camera
      # H_b2c = H_b2h *H_h2c 
      tr = H_b2c[0:3,3:4].T[0]
      quat = tfs.quaternions.mat2quat(H_b2c[0:3,0:3])                       # wxyz form
      
    quat_t = np.roll(quat, -1)                                              # xyzw

    
    return tr, quat_t                           

  @staticmethod # translate pose in base_link to dummy in ur robot
  def world_base_trans(position, orien, num):
        trans = Transformation()
        tr = [0, 0, 0]
        if num == 0:
            #transfer pose in base link to dummy
            quat_base = np.array([0.92388, 0, 0, 0.382683])
            
        elif num ==1:
            # transfer pose in dummy to base_link
            quat_base = np.array([0.92388, 0, 0, -0.382683])
            
        base_rot = trans.RotMatrix(quat_base)
        H_d2b = tfs.affines.compose(np.squeeze(tr), base_rot, [1, 1, 1])                 # dummy to base
        obj_rot = trans.RotMatrix(orien)
        H_b2o = tfs.affines.compose(np.squeeze(position), obj_rot, [1, 1, 1])                 # dummy to base
            
        H_b2o = np.dot(H_d2b,H_b2o)   
        translation = H_b2o[0:3,3:4].T[0]

        quat = tfs.quaternions.mat2quat(H_b2o[0:3,0:3])                  # wxyz form
        quat_t = np.roll(quat, -1)
            
        return translation, quat_t
  
# rotationVector = np.array([0.0955415182750808, 0.0475064330746053,0.0485482885665371])
# trans = Transformation()
# rota = trans.RotMatrix(rotationVector, rodrigues=True)
# print('rota', rota)