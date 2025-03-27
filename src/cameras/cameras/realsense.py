import cv2
# import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool  # You can customize the message type
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import threading
import os
import time
import numpy as np
import pickle

import pyrealsense2 as rs


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')

        self.declare_parameter('serial', "023322060631")
        self.declare_parameter('name', "realsense")
        
        self.rs_serial = self.get_parameter('serial').value
        self.name = self.get_parameter("name").value
        
        self.camera_fps = 30
        self.initialize_cameras()
        
        self.image_pub = self.create_publisher(Image, f'/realsense/{self.name}/im', 10) 
        self.depth_pub = self.create_publisher(Image, f'/realsense/{self.name}/depth', 10)
                
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/self.camera_fps, self.timer_callback) # might go back to 30hz -> see if 100 works better
        
    def initialize_cameras(self):
        W = 640
        H = 480

        all_detected_cameras = [dev.get_info(rs.camera_info.serial_number) for dev in rs.context().query_devices()]


        if self.rs_serial in all_detected_cameras:
            self.get_logger().info(f"Found real sense camera with serial number {self.rs_serial}")
            try:
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(self.rs_serial)
                config.enable_stream(rs.stream.depth, W, H, rs.format.z16, self.camera_fps)
                config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, self.camera_fps)

                profile = self.pipeline.start(config)
                depth_sensor = profile.get_device().first_depth_sensor()
                depth_scale = depth_sensor.get_depth_scale()        
                align_to = rs.stream.color
                self.align = rs.align(align_to)
            except Exception as e:
                self.get_logger().error(f"Failed to initialize camera {self.rs_serial}: {e}")
                exit()
        else:
            self.get_logger().error(f"Cannot find real sense camera with serial number {self.rs_serial}")
            exit()

        # cv2.namedWindow("Live RGB Image", cv2.WINDOW_NORMAL)


                    
    def get_rgb_msg(self, aligned_frames):
        color_frame = aligned_frames.get_color_frame()
        image_data = np.asanyarray(color_frame.get_data())


        # cv2.imshow("Live RGB Image", image_data)
        # cv2.waitKey(1)


        image_rgb = cv2.cvtColor(image_data, cv2.COLOR_BGRA2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")
        return image_msg

    def get_depth_msg(self, aligned_frames):
        aligned_depth_frame = aligned_frames.get_depth_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_image = depth_image.astype(np.float32)/1000
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        return depth_msg

    def timer_callback(self):
        self.get_logger().info(f"RS alive")
        start = time.time()
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        image_msg = self.get_rgb_msg(aligned_frames)
        depth_msg = self.get_depth_msg(aligned_frames)
        self.image_pub.publish(image_msg)
        self.depth_pub.publish(depth_msg)

        effective_hertz = 1/(time.time() - start)
        if effective_hertz < self.camera_fps-10:
            self.get_logger().warn(f"WARNING: Effective hz: {effective_hertz}")


    def destroy_node(self):
        self.pipeline.stop()
        self.get_logger().info("Destroyed RealSense Nodes")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    rs_node = RealSenseNode()
    try:
        rclpy.spin(rs_node)
    except KeyboardInterrupt:
        pass
    finally:
        rs_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__': 
    main()