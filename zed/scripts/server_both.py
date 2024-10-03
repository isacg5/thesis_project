#! /usr/bin/env python3
import socket
import rclpy
from zed.msg import Detection
from zed.msg import Evaluation
from zed.msg import Position
import ast
import numpy as np
import struct
#import cv2 as cv
HOST="10.0.0.1"
PORT=65433


def recv_msg(sock):
    raw_msglen = recvall(sock, 4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    return recvall(sock, msglen)

def recvall(sock, n):
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

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
                    data=recv_msg(conn)
                    #data= conn.recv(1024)
                    if not data:
                        continue
                    res=data.decode()
                    x=res.split("/")
                    #print(len(x))
                    print("Rec: ", x, "\n")
                    #print(" ")
                    if len(x)==7:
                        det.conf=float(x[1])
                        det.id=float(x[0])

                        position = ast.literal_eval(x[5])
                        #print(position)
                        det.position.x = position[0]
                        det.position.y = position[1]
                        det.position.z = position[2]
                        pub.publish(det)

                        ev.deep = -1 * position[2]
                        ev.conf = float(x[1])
                        ev.state = str(x[2])

                        key_pos = ast.literal_eval(x[3])
                        #print(type(key_pos[0]))
                        #print("POSES: ", pos)

                        pos = []
                        for i,j in key_pos:
                            new_pos = Position()
                            new_pos.x = i
                            new_pos.y = j
                            pos.append(new_pos)

                        #print(pos)
                        ev.poses = pos #incluir cambios que hay en el docker de spot

                        #print(x[4])
                        str_list = x[4].strip('[]')
                        str_list = str_list.split(',')

                        key_conf = []
                        for i in str_list:
                            x = i.strip()
                            if(x == 'nan'):
                                key_conf.append(-1.0)
                            else:
                                key_conf.append(float(x))

                        #print('k: ', key_conf)
                        #print("CONFS: ", pos_conf)
                        ev.poses_conf  = key_conf
                        pub_ev.publish(ev)

            except KeyboardInterrupt:
                print("Caught Keyboardinterrupt,exiting")

if __name__ == '__main__':
    if not main():
        sys.exit(1)
