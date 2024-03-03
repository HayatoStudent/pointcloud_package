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

class clusteringpointsSubPub(Node):


    def __init__(self):
        super().__init__("clusteringpoints")
        self.subscription = self.create_subscription(PointCloud2,'/points_no_ground',self.listener_callback,1)
        self.publisher = self.create_publisher(PointCloud2,'/clustering_point', 1)

    def listener_callback(self,msg): 
        """subscriber process"""

        ### process ###
        cloud = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z', "intensity"], skip_nans=True)
        pcd_data = np.delete(cloud, 3, 1)
        pcd_xyz_array = np.asarray(pcd_data)

        # filter range
        pcd_xyz_array=[np.delete(pcd_xyz_array, np.where(pcd_xyz_array[:, 0]<0), 0)] # fileter x range
        pcd_xyz_array = np.array(pcd_xyz_array)
        pcd_xyz_array = pcd_xyz_array.reshape(pcd_xyz_array.shape[1],pcd_xyz_array.shape[2])
        clustering = DBSCAN(eps=0.5, min_samples=100).fit(pcd_xyz_array) # DBSCAN clustering
        labels = clustering.labels_ # input label num
        cluster_object = []
        clusterpoints_num = []

        for lab_num in range(labels.max() + 1):
            # label num 2 points coordinate
            lab_points = pcd_xyz_array[np.where(labels == lab_num)]
            self.get_logger().info("/lab_points : {0}".format(lab_points))
            clusterpoints_num.append([lab_points.shape[0]])
            
            for i in range(len(lab_points)):
                cluster_object.append([lab_points[i][0], lab_points[i][1], lab_points[i][2], 0x00ff00])
                # color append

        self.get_logger().info("/cluster_object : {0}".format(len(cluster_object)))
        pcd_msg = point_cloud2.create_cloud(header=HEADER,fields=FIELDS,points=cluster_object)
        self.publisher.publish(pcd_msg)


def main(args=None):
    rclpy.init(args=args)         # rclpy init
    clusteringpoints_SubPub = clusteringpointsSubPub() # node
    rclpy.spin(clusteringpoints_SubPub)      # callback def
    rclpy.shutdown()               # rclpy close

if __name__ == '__main__':
    main()