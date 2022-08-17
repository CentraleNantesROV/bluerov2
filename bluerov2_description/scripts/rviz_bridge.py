#!/usr/bin/env python

'''
    Basic bridge to show the BlueROV2 in RViz with rotation and thruster efforts
'''

import rclpy
from geometry_msgs.msg import WrenchStamped, Pose, TransformStamped
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from rclpy.node import Node


class RVizRelay(Node):
    def __init__(self):
        super().__init__('rviz_relay')
        
        self.ns = self.get_namespace()[1:]
        self.br = TransformBroadcaster(self)
        
        # get ground truth
        self.pose_gt_sub = self.create_subscription(
            Pose,
            'pose_gt',
            self.store_pose,
            10)
        self.pose_gt = None
        self.tf = TransformStamped()
        self.tf.header.frame_id = 'world'
        self.tf.child_frame_id = self.ns + '/base_link_R'
        
        
        # get thruster inputs
        self.cmd_sub = []
        self.cmd = [None, None, None, None, None, None]
        self.wrench_pub = []
        self.wrench = WrenchStamped()
        
        for t in (0,1,2,3,4,5):
            self.cmd_sub.append(self.create_subscription(
                Float64, 
                f'cmd_thruster{t+1}', 
                lambda msg,idx=t: self.store_thrust(msg, idx),
                10))
            
            self.wrench_pub.append(self.create_publisher(WrenchStamped, f'wrench_{t+1}', 10))
            
        self.timer = self.create_timer(0.1, self.publish)        
        
        print('node created')
        
        
    def store_pose(self, msg):
        self.pose_gt = msg.position
        
    def store_thrust(self, msg, thruster):
        self.cmd[thruster] = msg.data
        
    def publish(self):
        
        now = self.get_clock().now().to_msg()
        
        if self.pose_gt is not None:
            self.pose2tf(now)
        
        self.wrench.header.stamp = now
        for t,cmd in enumerate(self.cmd):
            if cmd is not None:
                print(f'Received thrust for #{t+1}')
                self.thrust2wrench(cmd, t+1)
                

    def pose2tf(self, now):
        
        self.tf.header.stamp = now
        self.tf.transform.translation.x = self.pose_gt.x
        self.tf.transform.translation.y = self.pose_gt.y
        self.tf.transform.translation.z = self.pose_gt.z
        
        self.br.sendTransform(self.tf)
        
        
    def thrust2wrench(self, thrust, thruster):
        # build corresponding wrench
        self.wrench.header.frame_id = self.ns + f'/thruster_{thruster}'
        self.wrench.wrench.force.x = thrust
        
        self.wrench_pub[thruster-1].publish(self.wrench)
        
        

rclpy.init(args=None)

node = RVizRelay()

rclpy.spin(node)
rclpy.shutdown()
