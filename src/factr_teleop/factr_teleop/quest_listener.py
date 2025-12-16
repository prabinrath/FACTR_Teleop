#!/usr/bin/env python3

import socket
import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

def recvall(conn, size):
    """Receive exactly size bytes"""
    buffer = bytearray(size)
    view = memoryview(buffer)
    pos = 0
    while pos < size:
        read = conn.recv_into(view[pos:], size - pos)
        if not read:
            raise IOError("Connection closed")
        pos += read
    return bytes(buffer)

def read_int32(conn):
    """Read 4 bytes and unpack to int"""
    return struct.unpack("<I", recvall(conn, 4))[0]

def read_string(conn):
    """Read string with length prefix"""
    str_len = read_int32(conn)
    if str_len == 0:
        return ""
    return recvall(conn, str_len).decode("utf-8").rstrip("\x00")

def read_message(conn):
    """Read destination and data"""
    destination = read_string(conn)
    data_len = read_int32(conn)
    data = recvall(conn, data_len) if data_len > 0 else b""
    return destination, data

def decode_pose_stamped(data):
    """Decode PoseStamped: header + position + orientation"""
    offset = 0
    seq = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    stamp_secs = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    stamp_nsecs = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    
    frame_id_len = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    frame_id = data[offset:offset+frame_id_len].decode('utf-8') if frame_id_len > 0 else ""
    offset += frame_id_len
    
    # Position (3 doubles) + orientation (4 doubles)
    pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w = struct.unpack('<7d', data[offset:offset+56])
    
    return seq, stamp_secs, stamp_nsecs, frame_id, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w

def decode_ovr2ros_inputs(data):
    """Decode OVR2ROSInputs: 2 bools + 4 floats"""
    button_upper = struct.unpack('<?', data[0:1])[0]
    button_lower = struct.unpack('<?', data[1:2])[0]
    thumb_h, thumb_v, press_idx, press_mid = struct.unpack('<4f', data[2:18])
    return button_upper, button_lower, thumb_h, thumb_v, press_idx, press_mid

# Shared buffers
BUFFER_SIZE = 5
pose_buffer = deque(maxlen=BUFFER_SIZE)
input_buffer = deque(maxlen=BUFFER_SIZE)
running = True

def receiver_thread(conn):
    """Background thread to receive and buffer messages"""
    global running
    try:
        while running:
            destination, data = read_message(conn)
            
            if destination == "" or destination.startswith("__"):
                continue
            
            if 'right_hand_pose' in destination:
                decoded = decode_pose_stamped(data)
                pose_buffer.append((time.time(), decoded))
                    
            elif 'right_hand_inputs' in destination:
                decoded = decode_ovr2ros_inputs(data)
                input_buffer.append((time.time(), decoded))
                
    except Exception as e:
        print(f"Receiver error: {e}")
        running = False

def main():
    global running
    
    rclpy.init()
    node = Node('quest_listener')
    
    # Declare parameters
    node.declare_parameter('host', '192.168.1.108')
    node.declare_parameter('port', 10000)
    node.declare_parameter('output_hz', 30)
    
    # Get parameters
    HOST = node.get_parameter('host').value
    PORT = node.get_parameter('port').value
    OUTPUT_HZ = node.get_parameter('output_hz').value
    
    # Create publishers
    pose_pub = node.create_publisher(PoseStamped, 'right_hand_pose', 10)
    joy_pub = node.create_publisher(Joy, 'right_hand_inputs', 10)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    sock.bind((HOST, PORT))
    sock.listen(5)
    node.get_logger().info(f"Listening on {HOST}:{PORT}")
    
    conn, addr = sock.accept()
    node.get_logger().info(f"Connected by {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    
    # Start background receiver
    thread = threading.Thread(target=receiver_thread, args=(conn,), daemon=True)
    thread.start()
    
    node.get_logger().info("Waiting for data...")
    interval = 1.0 / OUTPUT_HZ
    
    try:
        while running and rclpy.ok():
            start = time.time()
            
            # Publish latest pose as PoseStamped
            if pose_buffer:
                ts, data = pose_buffer[-1]
                seq, secs, nsecs, frame, px, py, pz, ox, oy, oz, ow = data
                
                pose_msg = PoseStamped()
                pose_msg.header.stamp = node.get_clock().now().to_msg()
                pose_msg.header.frame_id = frame if frame else "world"
                pose_msg.pose.position.x = px
                pose_msg.pose.position.y = py
                pose_msg.pose.position.z = pz
                pose_msg.pose.orientation.x = ox
                pose_msg.pose.orientation.y = oy
                pose_msg.pose.orientation.z = oz
                pose_msg.pose.orientation.w = ow
                pose_pub.publish(pose_msg)
            
            # Publish latest input as Joy
            if input_buffer:
                ts, data = input_buffer[-1]
                btn_up, btn_low, th_h, th_v, pr_idx, pr_mid = data
                
                joy_msg = Joy()
                joy_msg.header.stamp = node.get_clock().now().to_msg()
                joy_msg.axes = [th_h, th_v, pr_idx, pr_mid]
                joy_msg.buttons = [int(btn_up), int(btn_low)]
                joy_pub.publish(joy_msg)
            
            # Sleep to maintain constant rate
            elapsed = time.time() - start
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        node.get_logger().info("Stopped")
    finally:
        running = False
        conn.close()
        sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
