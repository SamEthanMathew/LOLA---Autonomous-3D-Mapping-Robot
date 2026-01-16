import sys
import os
import time
import socket
import struct
import argparse
from rplidar import RPLidar

def run_sender(host, port, device_port):
    print(f"--- LiDAR TCP Sender ---")
    print(f"Target: {host}:{port}")
    print(f"Device: {device_port}")
    
    # 1. Connect to LiDAR
    try:
        lidar = RPLidar(device_port, timeout=5)
        info = lidar.get_info()
        print(f"LiDAR Connected: {info}")
        health = lidar.get_health()
        print(f"Health: {health}")
    except Exception as e:
        print(f"Failed to connect to LiDAR: {e}")
        return

    # 2. Connect to Server (Laptop)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print(f"Connecting to {host}:{port}...")
        sock.connect((host, port))
        print("Connected!")
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        lidar.stop()
        lidar.disconnect()
        return

    # 3. Stream Loop
    try:
        # iter_measurements(max_buf_meas=3000) for Pi buffer fix
        iterator = lidar.iter_measurements(max_buf_meas=3000)
        
        count = 0
        last_print = time.time()
        
        print("Streaming data...")
        
        for new_scan, quality, angle, distance in iterator:
            # Pack: Quality (1 byte uchar), Angle (4 byte float), Distance (4 byte float)
            # Total 9 bytes
            payload = struct.pack('<Bff', quality, angle, distance)
            try:
                sock.sendall(payload)
                count += 1
            except BrokenPipeError:
                print("Connection lost.")
                break
                
            if time.time() - last_print > 1.0:
                print(f"Sent {count} points/sec")
                count = 0
                last_print = time.time()
                
    except KeyboardInterrupt:
        print("Stopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Cleaning up...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        sock.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stream LiDAR data over TCP")
    parser.add_argument("--host", type=str, required=True, help="IP address of the laptop/server")
    parser.add_argument("--port", type=int, default=12345, help="Port to connect to (default: 12345)")
    parser.add_argument("--device", type=str, default='/dev/ttyUSB0', help="LiDAR serial port")
    
    args = parser.parse_args()
    
    run_sender(args.host, args.port, args.device)
