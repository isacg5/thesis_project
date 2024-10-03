import numpy as np
import cv2
from zed.msg import Evaluation
import rclpy
from rclpy.node import Node
import math

# Dimensiones de la imagen
width = 2000
height = 900

THRESHOLD_HIP_SHOULDER_STAND = 210 #20
THRESHOLD_KNEE_HIP_STAND = 190 #20
THRESHOLD_ANKLE_KNEE_STAND = 230 #20

THRESHOLD_HIP_SHOULDER_SIT = 200 #20
THRESHOLD_KNEE_HIP_SIT = -100 #20
THRESHOLD_ANKLE_KNEE_SIT = 50 #20

ANGLE_SIT_THRESHOLD = 140

# Crear una imagen en negro vacía
#img = np.ones((height, width, 3), dtype=np.uint8)*255

#names = ["nariz", "cuello", "hombro_dcho", "codo_dcho", "muneca_dcha", "hombro_izdo", "codo_izdo", "muneca_izda", "cadera_dcha", "rodilla_dcha", "tobillo_dcho", "cadera_izda", "rodilla_izda", "tobillo_izdo", "ojo_dcho", "ojo_izdo", "oreja_dcha", "oreja_izda"]
#pos = [(1270, 129), (1150, 369), (944, 350), (735, 44), (-1, -1), (1350, 386), (1460, 639), (1380, 801), (1030, 822), (-1, -1), (-1, -1), (1300, 840), (-1, -1), (-1, -1), (1200, 78), (1290, 73), (1070, 123), (-1, -1)]

class GetInfo(Node):
    def __init__(self):
        super().__init__('Get_pose_information')
        self.subscription = self.create_subscription(Evaluation, '/evaluation', self.evaluation_clbk, 10)
        self.counter = 0
        self.hip_shoulder_r = []
        self.hip_shoulder_l = []
        self.knee_hip_l = []
        self.knee_hip_r = []
        self.ankle_knee_r = []
        self.ankle_knee_l = []

    def distances(self, pos):
        if(pos[2] != [-1.0, -1.0] and pos[5] != [-1.0, -1.0] and pos[8] != [-1.0, -1.0] and pos[11] != [-1.0, -1.0] and pos[9] != [-1.0, -1.0] and pos[12] != [-1.0, -1.0] and pos[10] != [-1.0, -1.0] and pos[13] != [-1.0, -1.0]):
            hip_shoulder_y_r = pos[8][1] - pos[2][1]
            knee_hip_y_r = pos[9][1] - pos[8][1]
            ankle_knee_y_r = pos[10][1] - pos[9][1]

            hip_shoulder_y_l = pos[11][1] - pos[5][1]
            knee_hip_y_l = pos[12][1] - pos[11][1]
            ankle_knee_y_l = pos[13][1] - pos[12][1]
            return hip_shoulder_y_r, hip_shoulder_y_l, knee_hip_y_r, knee_hip_y_l, ankle_knee_y_r, ankle_knee_y_l
        else:
            return None, None, None, None, None, None
 
    def evaluate_angle(self, angle):
        if(angle > ANGLE_SIT_THRESHOLD):
            return "OTHER"
        else:
            return "SIT"

    def general_ev(self, shoulder, hip, knee, ankle):
        if(shoulder != [-1.0, -1.0] and hip != [-1.0, -1.0] and ankle != [-1.0, -1.0]):
            hip_shoulder_y = hip[1] - shoulder[1]
            knee_hip_y = knee[1] - hip[1]
            ankle_knee_y = ankle[1] - knee[1]

            hip_shoulder_x = hip[0] - shoulder[0]
            knee_hip_x = knee[0] - hip[0]
            ankle_knee_x = ankle[0] -knee[0]

            #print("\nHIP SHOULDER ", hip_shoulder_x, " ", hip_shoulder_y)
            #print("KNEE HIP ", knee_hip_x, " ", knee_hip_y)
            #print("ANKLE KNEE ", ankle_knee_x, " ", ankle_knee_y, "\n")

            if(abs(hip_shoulder_x) > abs(hip_shoulder_y)):
                #print("LAYINGGG")
                return "LAY"

            if(hip_shoulder_y > THRESHOLD_HIP_SHOULDER_STAND and knee_hip_y > THRESHOLD_KNEE_HIP_STAND and ankle_knee_y > THRESHOLD_ANKLE_KNEE_STAND): 
                #print("Person standing")
                return "STAND"

            elif(hip_shoulder_y > THRESHOLD_HIP_SHOULDER_SIT and knee_hip_y > THRESHOLD_KNEE_HIP_SIT and knee_hip_y < THRESHOLD_KNEE_HIP_STAND):
                #print("Person sitting")
                return "SIT"

            else:
                #print("Can't know!")
                return None
        else:
            #print("Not enough info")
            return None


    def evaluation_clbk(self, msg):
        pos = []
        for i in msg.poses:
            pos.append([i.x, i.y])

        result = []

        result.append(self.general_ev(pos[2], pos[8], pos[9], pos[10]))
        result.append(self.general_ev(pos[5], pos[11], pos[12], pos[13]))

        angle_l, angle_r = self.angle_hips(pos)

        if(angle_l != None):
            a = self.evaluate_angle(angle_l)
            if(a == "SIT"):
               result.append(a)
        if(angle_r != None):
            b = self.evaluate_angle(angle_r)
            if(b == "SIT"):
               result.append(b)

        dist_l, dist_r = self.nose_ankle(pos)
        result.append(dist_l)
        result.append(dist_r)

        filtered = [st for st in result if st is not None]
        if not filtered:
            common = "Unknown"
        else:
            cont = Counter(result)
            common = cont.most_common(1)[0][0]
        if(common != None):
            common = common.lower()
        final_result = common

        print("RESULT : ", result , "\n")
        print(final_result)


        '''
        if self.counter < 20:
            self.counter += 1
            hip_shoulder_y_r, hip_shoulder_y_l, knee_hip_y_r, knee_hip_y_l, ankle_knee_y_r, ankle_knee_y_l = self.distances(pos)
            if(hip_shoulder_y_r != None and hip_shoulder_y_l != None and knee_hip_y_r != None and knee_hip_y_l != None and ankle_knee_y_l != None and ankle_knee_y_r != None): 
                self.hip_shoulder_r.append(hip_shoulder_y_r)
                self.hip_shoulder_l.append(hip_shoulder_y_l)
                self.knee_hip_r.append(knee_hip_y_r)
                self.knee_hip_l.append(knee_hip_y_l)
                self.ankle_knee_r.append(ankle_knee_y_r)
                self.ankle_knee_l.append(ankle_knee_y_l)
            else:
                sum1 = sum(self.hip_shoulder_r) / 20
                sum2 = sum(self.hip_shoulder_l) / 20
                sum3 = sum(self.knee_hip_r) / 20
                sum4 = sum(self.knee_hip_l) / 20
                sum5 = sum(self.ankle_knee_r) / 20
                sum6 = sum(self.ankle_knee_l) / 20
          
                self.counter = 0
                self.hip_shoulder_r = []
                self.hip_shoulder_l = []
                self.knee_hip_r = []
                self.knee_hip_l = []
                self.ankle_knee_r = []
                self.ankle_knee_l = []

                print("Hip shoulder l: ", sum1, " Hip shoulder r: ", sum2, " Knee hip r: ", sum3, " Knee hip l: ", sum4, " Ankle knee r: ", sum5, " Ankle knee l: ", sum6)
  
        '''
    def get_angle(self, A, B, C):
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

    def angle_hips(self, pos):
        a1 = None
        a2 = None
        if(pos[5] != [-1.0, -1.0] and pos[11] != [-1.0, -1.0] and pos[12] != [-1.0, -1.0]):
            a1 = self.get_angle(pos[5], pos[11], pos[12])
            #print("Angle hips l ", a1)
        else:
            d = 0
            #print("No se puede obtener info de la cadera izquierda")

        if(pos[2] != [-1.0, -1.0] and pos[8] != [-1.0, -1.0] and pos[9] != [-1.0, -1.0]):
            a2 = self.get_angle(pos[2], pos[8], pos[9])
            #print("Angle hips r ", a2)
        else:
            d = 0
            #print("No se puede obtener info de la cadera derecha")

        return a1, a2


    def evaluate_nose_ankle_distance(self, dx, dy):
        dif = abs(dx - dy)
        dx = abs(dx)
        dy = abs(dy)
        #print("DIF: ", dif)
        if(dx < dy and dy > 700 and dif > 200):
            return "STAND"
        elif(dx < dy and dy > 300 and dy < 700 and dif > 200):
            return "SIT"
        elif(dx > dy and dif > 300):
            return "LAY"

    def nose_ankle(self, pos):
        a = None
        b = None
        # If the distance in x is smaller than the distance in y, it means the person is vertically located
        if(pos[0] != [-1.0, -1.0] and pos[13] != [-1.0, -1.0]):
            d1x, d1y = pos[0][0] - pos[13][0], pos[0][1] - pos[13][1]
            #print("Nose - Left Ankle ", d1x, d1y)
            a = self.evaluate_nose_ankle_distance(d1x, d1y)
        else:
            d = 0
            #print("No se puede obtener info del tobillo izquierdo")

        if(pos[0] != [-1.0, -1.0] and pos[10] != [-1.0, -1.0]):
            d2x, d2y = pos[0][0] - pos[10][0], pos[0][1] - pos[10][1]
            #print("Nose - Right Ankle ", d2x, d2y)
            b = self.evaluate_nose_ankle_distance(d2x, d2y)
        else:
            d = 0
            #print("No se puede obtener info del tobillo derecho")
        
        return a, b

    def ankle_dist(self, pos):
        if(pos[10] != [-1.0, -1.0] and pos[13] != [-1.0, -1.0]):
            d = math.sqrt((pos[0][0] - pos[13][0])**2 + (pos[10][1] - pos[13][1])**2)
            #print("Ankle dist ", d)
        else:
            d = 0
            #print("No se puede obtener info de los tobillos")

    def stand(self, pos):
        if(pos[2] != [-1.0, -1.0] and pos[5] != [-1.0, -1.0] and pos[8] != [-1.0, -1.0] and pos[11] != [-1.0, -1.0] and pos[9] != [-1.0, -1.0] and pos[12] != [-1.0, -1.0] and pos[10] != [-1.0, -1.0] and pos[13] != [-1.0, -1.0]):
            hip_shoulder_y_r = pos[8][1] - pos[2][1]
            knee_hip_y_r = pos[9][1] - pos[8][1]
            ankle_knee_y_r = pos[10][1] - pos[9][1]

            hip_shoulder_y_l = pos[11][1] - pos[5][1]
            knee_hip_y_l = pos[12][1] - pos[11][1]
            ankle_knee_y_l = pos[13][1] - pos[12][1]

            print("Hip shoulder r ", hip_shoulder_y_r)
            print("Knee hip r ", knee_hip_y_r)
            print("Ankle knee r ", ankle_knee_y_r)

            print("Hip shoulder l ", hip_shoulder_y_l) 
            print("Knee hip l ", knee_hip_y_l)
            print("Ankle knee l ", ankle_knee_y_l)

            if(hip_shoulder_y_r > THRESHOLD_HIP_SHOULDER_STAND and hip_shoulder_y_l > THRESHOLD_HIP_SHOULDER_STAND and knee_hip_y_r > THRESHOLD_KNEE_HIP_STAND and knee_hip_y_l > THRESHOLD_KNEE_HIP_STAND and ankle_knee_y_r > THRESHOLD_ANKLE_KNEE_STAND and ankle_knee_y_l > THRESHOLD_ANKLE_KNEE_STAND):
                print("Person standing")
                return True

            elif(hip_shoulder_y_r > THRESHOLD_HIP_SHOULDER_SIT and hip_shoulder_y_l > THRESHOLD_HIP_SHOULDER_SIT and knee_hip_y_r > THRESHOLD_KNEE_HIP_SIT and knee_hip_y_r < THRESHOLD_KNEE_HIP_STAND and knee_hip_y_l > THRESHOLD_KNEE_HIP_SIT and knee_hip_y_l < THRESHOLD_KNEE_HIP_STAND and ankle_knee_y_r > THRESHOLD_ANKLE_KNEE_SIT and ankle_knee_y_r < THRESHOLD_ANKLE_KNEE_STAND and ankle_knee_y_l > THRESHOLD_ANKLE_KNEE_SIT and ankle_knee_y_l < THRESHOLD_ANKLE_KNEE_STAND):
                print("Person sitting")
                return True
            else:
                print("Can't know!")
        else:
            print("Not enough info")

def main(args=None):
    rclpy.init(args=args)
    info = GetInfo()
    rclpy.spin(info)
    info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
