# -*- coding: utf-8 -*- #apply Japanese
from typing import List
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.time import Duration
from rclpy.context import Context 
from rclpy.node import Node
from rclpy.parameter import Parameter 
from std_msgs.msg import Int16 
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import numpy as np
import open3d as o3d 
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import random

# Frame ID
HEADER = Header(frame_id='/velodyne')

# PointCloud2 fields
FIELDS = [
    # (x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # (RGB)
    # red: 0xff0000, green:0x00ff00, blue: 0x0000ff
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]


class TrackingpointsSubPub(Node):


    def __init__(self):
        super().__init__("centerpoints")
        self.subscription = self.create_subscription(PointCloud2,'/points_no_ground',self.listener_callback,1)
        
        self.publisher = self.create_publisher(PointCloud2,'/tracking_points', 1)
        self.class_location = None # init frame
        self.frame_num = 0
        self.threshold = 0.65 # for tracking threshold

    def listener_callback(self,msg): 
        """subscriber process"""
        
        ### points 2 numpy process ###
        cloud = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z', "intensity"], skip_nans=True)
        pcd_data = np.delete(cloud, 3, 1)
        pcd_xyz_array = np.asarray(pcd_data)
        # filter range
        pcd_xyz_array=[np.delete(pcd_xyz_array, np.where((pcd_xyz_array[:, 0]<0) | (pcd_xyz_array[:, 2]<-1.98)), 0)] #x座標が0より小さい点は削除する,z座標が-1.98より小さい点は削除する
        pcd_xyz_array = np.array(pcd_xyz_array)
        pcd_xyz_array = pcd_xyz_array.reshape(pcd_xyz_array.shape[1],pcd_xyz_array.shape[2])
        
        clustering = DBSCAN(eps=0.5, min_samples=120).fit(pcd_xyz_array) # DBSCAN clustering
        labels = clustering.labels_ # input label num
        cluster_stock = [] # cluster points xyz
        center_stock = [] # center points xyz
        clusterpoints_num = [] # cluster num

        for lab_num in range(labels.max() + 1):

            # label num 2 points coordinate
            lab_points = pcd_xyz_array[np.where(labels == lab_num)]
            clusterpoints_num.append([lab_points.shape[0]])

            (x_min, y_min, z_min) = np.min(lab_points, axis=0) # xyz min
            (x_max, y_max, z_max) = np.max(lab_points, axis=0) # xyz max
            
            # random color append
            color = ''.join([random.choice('01') for j in range(6)])

            # catch center xyz
            half_x = (x_max+x_min)/2
            half_y = (y_max+y_min)/2
            half_z = (z_max+z_min)/2

            center_stock.append([half_x, half_y, half_z, lab_num, color])
            for i in range(len(lab_points)):
                cluster_stock.append([lab_points[i][0], lab_points[i][1], lab_points[i][2], lab_num, color]) 

        cluster_np = np.asarray(cluster_stock)
        center_np = np.asarray(center_stock)

        """tracking process"""

        if self.frame_num != 0 and bool(self.class_location and center_stock): # init frame or If not before
            center_stock = [] # reset cluster points xyz
            cluster_stock = [] # reset center points xyz
            for lab_num in range(labels.max() + 1):

                # label num 2 points coordinate
                lab_points = pcd_xyz_array[np.where(labels == lab_num)]
                clusterpoints_num.append([lab_points.shape[0]])
                (x_min, y_min, z_min) = np.min(lab_points, axis=0) 
                (x_max, y_max, z_max) = np.max(lab_points, axis=0) 
                color = ''.join([random.choice('01') for j in range(6)])

                half_x = (x_max+x_min)/2
                half_y = (y_max+y_min)/2
                half_z = (z_max+z_min)/2

                distance = [] # clustering distance list
               
                for class_num in range(len(self.class_location)):
                    """"""
                    new_xy = np.array([center_np[lab_num][0], center_np[lab_num][1]], dtype='float64')
                    old_xy = np.array([self.class_location[class_num][0], self.class_location[class_num][1]], dtype='float64')
                    distance.append(np.linalg.norm(new_xy-old_xy)) # distance eval

                else:
                    min_index = distance.index(min(distance)) # dis min index

                    if min(distance) > self.threshold:
                        center_stock.append([half_x, half_y, half_z, lab_num, color])
                        for i in range(len(lab_points)):
                            cluster_stock.append([lab_points[i][0], lab_points[i][1], lab_points[i][2], lab_num, color])   

                    # clustering visualization
                    for i in range(len(lab_points)):
                        cluster_stock.append([lab_points[i][0], lab_points[i][1], lab_points[i][2], lab_num, self.class_location[min_index][4]])
                    else:
                        center_stock.append([half_x, half_y, half_z, lab_num, self.class_location[min_index][4]])    
            
        # list info input
        center_object = center_stock
        cluster_object = cluster_stock
        self.class_location = center_object

        if center_object and cluster_object:
            self.get_logger().info("center xyz : {0}".format(center_object))

        pcd_msg = point_cloud2.create_cloud(header=HEADER,fields=FIELDS,points=cluster_object)

        self.frame_num += 1
        
        self.publisher.publish(pcd_msg)

def main(args=None):
    rclpy.init(args=args)          # rclpy init
    Trackingpoints_SubPub = TrackingpointsSubPub() # node
    rclpy.spin(Trackingpoints_SubPub)      # callback def
    TrackingpointsSubPub.destory_node()   # node brake
    rclpy.shutdown()               # rclpy close

if __name__ == '__main__':
    main()