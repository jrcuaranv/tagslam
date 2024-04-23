#!/usr/bin/env python
"""
It extracts the poses of the observed april tags (with PnP algorithm) and publish them in a path topic
the header.seq of each pose corresponds to the tag_id
These poses correspond to the transformation from the observed tag wrt to the left camera
so that can be used as a betweenFactor.
It also extracts stereo and monocular (generic projection) factors for the tag centers.
These projection factors are merged in a path topic for convenience.
"""


from nav_msgs.msg import Path
from cv_bridge import CvBridge
import numpy as np
import apriltag
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge


class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()
        img_topic = rospy.get_param("/stereo_cam/topic")
        self.image_sub = rospy.Subscriber(img_topic, CompressedImage, self.image_callback)
        self.tagpose_pub = rospy.Publisher('/tagpose', Path, queue_size=10)
        self.projection_factors_pub = rospy.Publisher('/projection_factors', Path, queue_size=10)
        
        self.img_count = 0
        self.skip_n_img = rospy.get_param("/stereo_cam/skip_img")
        width = rospy.get_param("/apriltag_size") # width size of apriltag [m]
        self.pts_3d = np.array([[-width/2, width/2, 0.0],
                            [width/2, width/2, 0.0],
                            [width/2, -width/2, 0.0],
                            [-width/2, -width/2, 0.0]])

        # Left camera intrinsics

        fx = rospy.get_param("/stereo_cam/left/intrinsics/fx")
        fy = rospy.get_param("/stereo_cam/left/intrinsics/fy")
        cx = rospy.get_param("/stereo_cam/left/intrinsics/cx")
        cy = rospy.get_param("/stereo_cam/left/intrinsics/cy")
        k1 = rospy.get_param("/stereo_cam/left/dist/k1")
        k2 = rospy.get_param("/stereo_cam/left/dist/k2")
        r1 = rospy.get_param("/stereo_cam/left/dist/r1")
        r2 = rospy.get_param("/stereo_cam/left/dist/r2")

        self.K_left = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.dist_coeffs_left = np.array([k1, k2, r1, r2]).reshape(4,1)

        # Right camera intrinsics

        fx = rospy.get_param("/stereo_cam/right/intrinsics/fx")
        fy = rospy.get_param("/stereo_cam/right/intrinsics/fy")
        cx = rospy.get_param("/stereo_cam/right/intrinsics/cx")
        cy = rospy.get_param("/stereo_cam/right/intrinsics/cy")
        k1 = rospy.get_param("/stereo_cam/right/dist/k1")
        k2 = rospy.get_param("/stereo_cam/right/dist/k2")
        r1 = rospy.get_param("/stereo_cam/right/dist/r1")
        r2 = rospy.get_param("/stereo_cam/right/dist/r2")

        self.K_right = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.dist_coeffs_right = np.array([k1, k2, r1, r2]).reshape(4,1)

        # Computing transformation from left camera to right camera
        t_body_left = np.array(rospy.get_param("/stereo_cam/left/body_P_cam_pos")).reshape(3,1)
        q_body_left = rospy.get_param("/stereo_cam/left/body_P_cam_quat")
        R_body_left = tf.transformations.quaternion_matrix(q_body_left)[:3,:3]
        T_body_left = np.block([[R_body_left, t_body_left],[0,0,0,1]])
        
        t_body_right = np.array(rospy.get_param("/stereo_cam/right/body_P_cam_pos")).reshape(3,1)
        q_body_right = rospy.get_param("/stereo_cam/right/body_P_cam_quat")
        R_body_right = tf.transformations.quaternion_matrix(q_body_right)[:3,:3]
        T_body_right = np.block([[R_body_right, t_body_right],[0,0,0,1]])

        T_right_body = np.linalg.inv(T_body_right)
        self.T_rl = np.matmul(T_right_body, T_body_left)
        self.T_lr = np.linalg.inv(self.T_rl)
        print("Waiting for image topics...")
        rospy.spin()

    def image_callback(self, msg):
        self.img_count += 1
        try:
            if(self.img_count%self.skip_n_img == 0):
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image_color = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)
                timestamp = msg.header.stamp

                m, n = cv_image.shape # gray image
                left = cv_image[:,0:(n//2)]
                right = cv_image[:,(n//2):]
                
                tags_left = self.detector.detect(left)
                tags_right = self.detector.detect(right)
                self.extract_tags_poses (tags_left, tags_right, timestamp)
                self.extract_projection_factors (tags_left, tags_right, timestamp)
        except Exception as e:
            print(e)
    def extract_projection_factors(self, tags_left, tags_right, timestamp):
        """
        Extracts stereo factors and generic projection factors depending on whether or not
        the tags appear on left and right image or just in a single one.
        """
        tag_left_list = []
        factors = []
        success = False
        stereo_factors = False
        projection_factor = False
        # getting stereo factors
        for tag_l in tags_left:
            for tag_r in tags_right:
                if tag_l.tag_id==tag_r.tag_id:
                    stereo_factors = True
                    # creating stereo factors
                    factors.append(PoseStamped(header=rospy.Header(seq = tag_l.tag_id, stamp=timestamp, frame_id="world"),
                                             pose=Pose(position=Point(tag_l.center[0], tag_r.center[0], tag_l.center[1]),
                                                       orientation=Quaternion(0,0,0,2)))) # qw indicates the type of factor (2 for stereo)
        # getting generic projection factors (monocular)
        tags_left_dic = {}
        tags_right_dic = {}
        for tag in tags_left:
            tags_left_dic['tag'+str(tag.tag_id)] = [tag.tag_id, tag.center[0], tag.center[1]]
        for tag in tags_right:
            tags_right_dic['tag'+str(tag.tag_id)] = [tag.tag_id, tag.center[0], tag.center[1]]
        left_cam_id = 0
        right_cam_id = 1
        for tag_key in tags_left_dic:
            if tag_key not in tags_right_dic and tags_left_dic[tag_key][0]:
                factors.append(PoseStamped(header=rospy.Header(seq = tags_left_dic[tag_key][0], stamp=timestamp, frame_id="world"),
                                             pose=Pose(position=Point(left_cam_id, tags_left_dic[tag_key][1], tags_left_dic[tag_key][2]),
                                                       orientation=Quaternion(0,0,0,3)))) # qw indicates the type of factor (3 for generic projection
                projection_factor = True
                #print("New projection factor left cam tag id:", tags_left_dic[tag_key][0])
        for tag_key in tags_right_dic:
            if tag_key not in tags_left_dic and tags_right_dic[tag_key][0]:
                factors.append(PoseStamped(header=rospy.Header(seq = tags_right_dic[tag_key][0], stamp=timestamp, frame_id="world"),
                                             pose=Pose(position=Point(right_cam_id, tags_right_dic[tag_key][1], tags_right_dic[tag_key][2]),
                                                       orientation=Quaternion(0,0,0,3)))) # qw indicates the type of factor (3 for generic projection
                projection_factor = True
                #print("New projection factor right cam tag id:", tags_right_dic[tag_key][0])

        path_msg = Path() 
        path_msg.poses = factors
        path_msg.header.stamp = timestamp
        path_msg.header.frame_id = 'leftcam' #doesn't really matter
        if projection_factor or stereo_factors:
            self.projection_factors_pub.publish(path_msg)
            print("New projection factor published")
    
    def extract_tags_poses(self, tags_left, tags_right, timestamp):
        """
        Getting tag poses just for initialization 
        """
        
        tag_left_list = []
        tag_poses = []
        success = False
        
        for tag_l in tags_left:

            tag_left_list.append(tag_l.tag_id)
            success, rotation_vector, translation_vector = cv2.solvePnP(self.pts_3d, tag_l.corners, self.K_left, self.dist_coeffs_left, flags=0)
            t_lw = translation_vector #from tag to camera
            r_lw = rotation_vector
            R_lw, xxx = cv2.Rodrigues(rotation_vector) # rot.vector to Rotation matrix
            T_lw = np.eye(4)
            T_lw[0:3,0:3] = R_lw
            # T_lw[0:3,3] = t_lw.flatten() translation does not mater here
            q_lw = tf.transformations.quaternion_from_matrix(T_lw)
            left_between_factor = '6,'+str(tag_l.tag_id) + ',' + str(t_lw[0].item()) + ',' + str(t_lw[1].item())+ ',' + str(t_lw[2].item()) + ',' + str(r_lw[0].item()) + ',' + str(r_lw[1].item()) + ',' + str(r_lw[2].item())
            if success:
                tag_poses.append(PoseStamped(header=rospy.Header(seq = tag_l.tag_id, stamp=timestamp, frame_id="world"),
                                             pose=Pose(position=Point(t_lw[0].item(),t_lw[1].item(),t_lw[2].item()), orientation=Quaternion(q_lw[0].item(),q_lw[1].item(),q_lw[2].item(),q_lw[3].item()))))
            
        for tag_r in tags_right:
            if tag_r.tag_id in tag_left_list: #skip repetitive factors
                continue
            
            success, rotation_vector, translation_vector = cv2.solvePnP(self.pts_3d, tag_r.corners, self.K_right, self.dist_coeffs_right, flags=0)
            
            t_rw = translation_vector #from tag to camera
            r_rw = rotation_vector
            R_rw, xxx = cv2.Rodrigues(rotation_vector) # rot.vector to Rotation matrix
            T_rw = np.block([[R_rw, t_rw.reshape(3,1)],[0, 0, 0, 1]])
            # getting tagpose wrt to left camera
            T_lw = np.matmul(self.T_lr, T_rw)
            t_lw = T_lw[0:3, 3]
            R_lw = T_lw[0:3, 0:3]
            r_lw, __ =  cv2.Rodrigues(R_lw)
            q_lw = tf.transformations.quaternion_from_matrix(T_lw)
            if success:
                tag_poses.append(PoseStamped(header=rospy.Header(seq = tag_r.tag_id, stamp=timestamp, frame_id="world"),
                                             pose=Pose(position=Point(t_lw[0].item(),t_lw[1].item(),t_lw[2].item()), orientation=Quaternion(q_lw[0].item(),q_lw[1].item(),q_lw[2].item(),q_lw[3].item()))))
        path_msg = Path() 
        path_msg.poses = tag_poses
        path_msg.header.stamp = timestamp
        path_msg.header.frame_id = 'leftcam'
        if success:
            self.tagpose_pub.publish(path_msg)
            print("New tag poses were published")

if __name__ == '__main__':
    try:
        image_subscriber = ImageSubscriber()
    except rospy.ROSInterruptException:
        pass


