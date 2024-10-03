#! /usr/bin/env python3
import socket
import rclpy
from zed.msg import Detection
from zed.msg import Evaluation
from zed.msg import Position
import ast
import numpy as np
#import cv2 as cv
HOST="10.0.0.1"
PORT=65433

def main():
    rclpy.init()
    node = rclpy.create_node('sock')
    pub = node.create_publisher(Detection, "position", 20)
    pub_ev = node.create_publisher(Evaluation, "evaluation", 20)
    det = Detection()
    ev = Evaluation()
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
                    print(x)
                    print(" ")
                    if len(x)>1:
                        det.conf=float(x[1])
                        det.id=float(x[0])
                        position = []
                        b = x[5].replace('[', '').replace(']', '').strip()
                        position_str = b.split()
                        position = [float(num) for num in position_str]
                        det.position.x = position[0]
                        det.position.y = position[1]
                        det.position.z = position[2]
                        pub.publish(det)

                        ev.deep = -1 * position[2]
                        ev.conf = float(x[1])
                        ev.state = str(x[2])
                        pos = []
                        a = x[3].replace('[', '').replace(']', '').strip()
                        nums_str = a.split()
                        nums = [float(num) for num in nums_str]
                        for i in range(0, len(nums), 2):
                            new_pos = Position()
                            new_pos.x = nums[i]
                            new_pos.y = nums[i+1]
                            pos.append(new_pos)
                        #print("POSES: ", pos)
                        ev.poses = pos #incluir cambios que hay en el docker de spot
                        pos_conf = []
                        c = x[4].replace('[', '').replace(']', '').strip()
                        conf_str = c.split()
                        pos_conf = [float(num) for num in conf_str]
                        #print("CONFS: ", pos_conf)
                        ev.poses_conf  = pos_conf
                        pub_ev.publish(ev)

            except KeyboardInterrupt:
                print("Caught Keyboardinterrupt,exiting")

if __name__ == '__main__':
    if not main():
        sys.exit(1)
