#! /usr/bin/env python3
import socket
import rclpy
from zed.msg import Detection
import ast
import numpy as np
#import cv2 as cv
HOST="10.0.0.1"
PORT=65433

def main():
    rclpy.init()
    node = rclpy.create_node('sock')
    pub = node.create_publisher(Detection, "position", 20)
    det = Detection()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST,PORT))
        s.listen()
        print("before the accept")
        conn, addr = s.accept()
        with conn:
            try:
                print("connecte")
                while True:
                    data= conn.recv(1024)
                    if not data:
                        continue
                    res=data.decode()
                    x=res.split("/")
                    #print(len(x))
                    print(res)
                    #if len(x)>1:
                        #print(x[2])
                        #det.conf=float(x[0])
                        #det.label=int(x[1])
                        #det.position.x= float(x[2])
                        #det.position.y=float(x[3])
                        #det.position.z=float(x[4])
                        #pub.publish(det)
            except KeyboardInterrupt:
                print("Caught Keyboardinterrupt,exiting")

if __name__ == '__main__':
    if not main():
        sys.exit(1)
