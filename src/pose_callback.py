#!usr/bin/env python3

import rospy
import os
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import pandas as pd

save_path = os.path.dirname(__file__)
csv_file_name_1 = "sim_data_global.csv"
csv_file_name_2 = "18_12_data_encoder_rear.csv"
csv_file_name_4 = "18_12_data_encoder_front.csv"
csv_file_name_3 = "sim_data_gps.csv"


def pose_callback(data):
    pose_data = Pose()
    pose_data.position.x = data.pose.pose.position.x
    pose_data.position.y = data.pose.pose.position.y
    pose_data.position.z = 0.0
    pose_data.orientation.x = 0.0
    pose_data.orientation.y = 0.0
    pose_data.orientation.z = 0.0
    pose_data.orientation.w = 0.0

    a = pose_data.position.x
    b = pose_data.position.y

    df = pd.DataFrame([[a,b]])
    return df

def odom_callback(data):
    odom_data = Odometry()
    odom_data.header.stamp.nsecs = data.header.stamp.nsecs
    odom_data.pose.pose.position.x = data.pose.pose.position.x
    odom_data.pose.pose.position.y = data.pose.pose.position.y
    odom_data.pose.pose.position.z = 0.0
    odom_data.pose.pose.orientation.x = 0.0
    odom_data.pose.pose.orientation.y = 0.0
    odom_data.pose.pose.orientation.z = 0.0
    odom_data.pose.pose.orientation.w = 0.0

    x_pos = odom_data.pose.pose.position.x
    y_pos = odom_data.pose.pose.position.y

    df = pd.DataFrame([[x_pos,y_pos]])
    return df

if __name__ == '__main__':
    try:
        rospy.init_node('pose_callback')
        while not rospy.is_shutdown():
            odom_global = rospy.wait_for_message('/odometry/global', Odometry)
            # odom_encoder_front = rospy.wait_for_message('/odom_encoder/front', Odometry)
            # odom_encoder_rear = rospy.wait_for_message('/odom_encoder/rear', Odometry)
            odom_gps = rospy.wait_for_message('/odometry/gps', Odometry)
            df1 = pose_callback(odom_global) 
            # df2 = pose_callback(odom_encoder_front)
            # df4 = pose_callback(odom_encoder_rear)
            df3 = pose_callback(odom_gps)
            f1 =  open(save_path+str(csv_file_name_1), 'a')
            # f2 =  open(save_path+str(csv_file_name_2), 'a')
            f3 =  open(save_path+str(csv_file_name_3), 'a')
            # f4 =  open(save_path+str(csv_file_name_4), 'a')

            np.savetxt(f1, df1, delimiter = ',')
            # np.savetxt(f2, df2, delimiter = ',')
            np.savetxt(f3, df3, delimiter = ',')
            # np.savetxt(f4, df4, delimiter = ',')

        f1.close()
        # f2.close()
        f3.close()
        # f4.close()

    except rospy.ROSInterruptException:
        pass