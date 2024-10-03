#! /usr/bin/env python3
import socket
import rclpy
from zed.msg import Detection
import ast
import numpy as np
import contextvars
import math

#import cv2 as cv
HOST="10.0.0.1"
PORT=65433

# Dimensiones de la imagen
width = 2000
height = 900

THRESHOLD_HIP_SHOULDER = 20
THRESHOLD_KNEE_HIP = 20
THRESHOLD_ANKLE_KNEE = 20

def guess(loc):
    # Crear una imagen en negro vac??a
    img = np.ones((height, width, 3), dtype=np.uint8)*255

    names = ["nariz", "cuello", "hombro_dcho", "codo_dcho", "muneca_dcha", "hombro_izdo", "codo_izdo", "muneca_izda", "cadera_dcha", "rodilla_dcha", "tobillo_dcho", "cadera_izda", "rodilla_izda", "tobillo_izdo", "ojo_dcho", "ojo_izdo", "oreja_dcha", "oreja_izda"]
    #pos = [(1270, 129), (1150, 369), (944, 350), (735, 44), (-1, -1), (1350, 386), (1460, 639), (1380, 801), (1030, 822), (-1, -1), (-1, -1), (1300, 840), (-1, -1), (-1, -1), (1200, 78), (1290, 73), (1070, 123), (-1, -1)]
    pos = []
    a = loc.replace('[', '').replace(']', '').strip()
    nums_str = a.split()
    nums = [float(num) for num in nums_str]
    pos = [(nums[i], nums[i+1]) for i in range(0, len(nums), 2)]

    # Color del punto (en BGR)
    color = (0, 0, 0)  # Blanco

#    for i in range(len(names)):
#        cv2.circle(img, pos[i], 3, color, -1)  # Radio del c??rculo: 3, grosor: -1 (relleno)
#        texto(names[i], pos[i])

    angle_hips(pos)
    nose_ankle(pos)
    ankle_dist(pos)
    stand(pos)

    # Mostrar la imagen en una ventana de OpenCV
#    cv2.imshow('Imagen con punto en el centro', img)
#    cv2.waitKey(0)
#    cv2.destroyAllWindows()


        
def get_angle(A, B, C):
    BA = np.array([A[0] - B[0], A[1] - B[1]])
    BC = np.array([C[0] - B[0], C[1] - B[1]])

    # Dot product
    dot_product = np.dot(BA, BC)

    # Magnitudes
    magnitude_BA = np.linalg.norm(BA)
    magnitude_BC = np.linalg.norm(BC)

    # Cosine of the angle
    cos_angle = dot_product / (magnitude_BA * magnitude_BC)

    # Angle in radians
    angle_radians = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip to handle floating point errors

    # Angle in degrees
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees

def angle_hips(pos):
    if(pos[5] != (-1, -1) and pos[11] != (-1, -1) and pos[12] != (-1, -1)):
         a1 = get_angle(pos[5], pos[11], pos[12])
         print(a1)
    else:
         print("No se puede obtener info de la izquierda")

    if(pos[2] != (-1, -1) and pos[8] != (-1, -1) and pos[9] != (-1, -1)):
         a2 = get_angle(pos[2], pos[8], pos[9])
         print(a2)
    else:
         print("No se puede obtener info de la derecha")

def nose_ankle(pos):
    if(pos[0] != (-1, -1) and pos[13] != (-1, -1)):
        d1x, d1y = pos[0][0] - pos[13][0], pos[0][1] - pos[13][1]
        print(d1x, d1y)
    else:
        print("No se puede obtener info de la izquierda")

    if(pos[0] != (-1, -1) and pos[10] != (-1, -1)):
        d2x, d2y = pos[0][0] - pos[10][0], pos[0][1] - pos[10][1]
        print(d2x, d2y)
    else:
        print("No se puede obtener info de la derecha")


def ankle_dist(pos):
    if(pos[10] != (-1, -1) and pos[13] != (-1, -1)):
        d = math.sqrt((pos[0][0] - pos[13][0])**2 + (pos[10][1] - pos[13][1])**2)
        print(d)
    else:
        print("No se puede obtener info de los tobillos")

def stand(pos):
    if(pos[2] != (-1, -1) and pos[5] != (-1, -1) and pos[8] != (-1, -1) and pos[11] != (-1, -1) and pos[9] != (-1, -1) and pos[12] != (-1, -1) and pos[10] != (-1, -1) and pos[13] != (-1, -1)):
        hip_shoulder_y_r = pos[8][1] - pos[2][1]
        knee_hip_y_r = pos[9][1] - pos[8][1]
        ankle_knee_y_r = pos[10][1] - pos[9][1]

        hip_shoulder_y_l = pos[11][1] - pos[5][1]
        knee_hip_y_l = pos[12][1] - pos[11][1]
        ankle_knee_y_l = pos[13][1] - pos[12][1]


        if(hip_shoulder_y_r > THRESHOLD_HIP_SHOULDER and hip_shoulder_y_l > THRESHOLD_HIP_SHOULDER and knee_hip_y_r > THRESHOLD_KNEE_HIP and knee_hip_y_l > THRESHOLD_KNEE_HIP and ankle_knee_y_r > THRESHOLD_ANKLE_KNEE and ankle_knee_y_l > THRESHOLD_ANKLE_KNEE):
            print("Person standing")
            return True
        else:
            print("Can't know")
    else:
        print("Not enough info")



def texto(text, coord):
    global img
    x = coord[0]
    y = coord[1]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    font_thickness = 1
    text_color = (0, 0, 0)  # Blanco
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)

    text_x = x + 10  # Ajustar posici  n del texto
    text_y = y + text_size[1] // 2  # Ajustar posici  n del texto

    cv2.putText(img, text, (text_x, text_y), font, font_scale, text_color, font_thickness)



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
                    if(len(x) > 4):
                        guess(x[3])
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
