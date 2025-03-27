import cv2
import pyzed.sl as sl
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

class ZedNode(Node):
    def __init__(self):
        super().__init__('zed_node')

        self.declare_parameter('serial', 22176523)
        self.declare_parameter('name', "zed")
        
        self.zed_serial = self.get_parameter('serial').value
        self.name = self.get_parameter("name").value
        
        self.camera_fps = 30
        self.initialize_cameras()
        
        self.image_pub_left = self.create_publisher(Image, f'/zed/{self.name}/im_left', 10) 
        self.image_pub_right = self.create_publisher(Image, f'/zed/{self.name}/im_right', 10) 
        self.depth_pub = self.create_publisher(Image, f'/zed/{self.name}/depth', 10)
        
        self.trackers = {}
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/self.camera_fps, self.timer_callback) # might go back to 30hz -> see if 100 works better

        
    def initialize_cameras(self):
        all_detected_cameras = sl.Camera.get_device_list()
        
        init_params = sl.InitParameters()
        
        init_params.camera_resolution = sl.RESOLUTION.HD720 #sl.RESOLUTION.HD720
        init_params.camera_fps = self.camera_fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE #sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.sensors_required = True
        
        for camera in all_detected_cameras:
            self.get_logger().info(f"Found camera with serial number {camera.serial_number}")
            if camera.serial_number == self.zed_serial:
                self.zed = sl.Camera()
                status = self.zed.open(init_params)
                if status != sl.ERROR_CODE.SUCCESS:
                    print("Camera Open : "+repr(status)+". Exit program.")
                    exit()
                else:
                    self.zed.set_camera_settings(sl.VIDEO_SETTINGS.LED_STATUS, True)
                    self.get_logger().info("ZED Opened Successfully")
                    
    
    def get_rgb_msg(self, view):
        image = sl.Mat()
        self.zed.retrieve_image(image, view)
        image_data = image.get_data()
        image_rgb = cv2.cvtColor(image_data, cv2.COLOR_BGRA2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")
        return image_msg

    def get_depth_msg(self):
        depth = sl.Mat()
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        depth_msg = self.bridge.cv2_to_imgmsg(depth.get_data(), encoding="32FC1")
        return depth_msg

    def timer_callback(self):
        # self.get_logger().info(f"ZED alive")

        start = time.time()
        err = self.zed.grab()
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to grab frame: {err}")

        image_msg_left = self.get_rgb_msg(sl.VIEW.LEFT)
        image_msg_right = self.get_rgb_msg(sl.VIEW.RIGHT)
        depth_msg = self.get_depth_msg()    
        self.image_pub_left.publish(image_msg_left)
        self.image_pub_right.publish(image_msg_right)
        self.depth_pub.publish(depth_msg)
    
        effective_hertz = 1/(time.time() - start)
        if effective_hertz < self.camera_fps-10:
            self.get_logger().warn(f"WARNING: Effective hz: {effective_hertz}")

    def destroy_node(self):
        self.zed.close()
        self.get_logger().info("Destroyed ZED Nodes")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    zed_node = ZedNode()
    try:
        rclpy.spin(zed_node)
    except KeyboardInterrupt:
        pass
    finally:
        zed_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__': 
    main()