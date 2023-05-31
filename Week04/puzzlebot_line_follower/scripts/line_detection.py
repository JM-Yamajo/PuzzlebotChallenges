#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
import numpy as np
from std_msgs.msg import UInt8, Float32
from sensor_msgs.msg import Image
from puzzlebot_line_follower.msg import SetPoint

class ImageProcessing:

    def __init__(self):
        self.image = None
        self.bridge = cv_bridge.CvBridge()

        self.lower_red = np.array([0, 100, 100])
        self.upper_red = np.array([10, 255, 255])

        self.lower_yellow = np.array([15, 100, 100])
        self.upper_yellow = np.array([35, 255, 255])

        self.lower_green = np.array([40, 100, 100])
        self.upper_green = np.array([80, 255, 255])

        self.lower = np.array([0, 0, 0])
        self.upper = np.array([180, 255, 20])
        self.angle = 0.0

        self.roi_upper = 0.86
        self.roi_lower = 0.99
        self.roi_left = 0.35
        self.roi_right = 0.65

        # Escalado de la imagen recibida
        self.frame_Height = 480
        self.frame_Width = 640

        # Tamano de la region de interes
        self.roi_size = [int(self.roi_upper * self.frame_Height), int(self.roi_lower * self.frame_Height),
                        int(self.roi_left * self.frame_Width), int(self.roi_right * self.frame_Width - 1)]

        # Mensaje Personalizado de Salida
        self.target = SetPoint()
        self.target.orientation = 0.0
        self.target.id = 1

        # Publishers y Subscribers del Nodo
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.setpoint_pub = rospy.Publisher("/set_point", SetPoint, queue_size=10)

    # FUNCION DETECCION DE ANGULOS:
    def line_following(self, img):
        b_frame_H = 240
        b_frame_W = 320

        b_roi_upper = 0.50
        b_roi_lower = 0.99

        b_roi_left = 0.25
        b_roi_right = 0.75

        b_frame_Height = 480
        b_frame_Width = 640

        img_inv = cv2.rotate(img, cv2.ROTATE_180)
        resized = cv2.resize(img_inv, (b_frame_Width, b_frame_Height))
        roi_size = [int(self.roi_upper * self.frame_Height), int(self.roi_lower * self.frame_Height),
                    int(self.roi_left * self.frame_Width), int(self.roi_right * self.frame_Width)]

        frame = resized[roi_size[0]:roi_size[1], roi_size[2]:roi_size[3]]
        frame_H, frame_W, _ = frame.shape
        frame = cv2.resize(frame, (frame_W, frame_H))

        bin_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(bin_img, threshold1=30, threshold2=50)

        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

        if lines is not None:
            # Resultados ya se encuentran en radianes:
            lines_angles = np.arctan2(lines[:, :, 3] - lines[:, :, 1], lines[:, :, 2] - lines[:, :, 0])
            unique_angles, counts = np.unique(np.round(lines_angles / 7), return_counts=True)
            dominant_angle = (unique_angles[np.argmax(counts)]) * 7
            dominant_lines = lines[np.abs(lines_angles - dominant_angle) <= 10]

            points = dominant_lines.reshape(-1, 2)
            coefficients = np.polyfit(points[:, 0], points[:, 1], 1)

            x = max(min(np.round((80 - coefficients[1]) / coefficients[0], 2), 320), 0)

            # Angulo ya en radianes (2 decimales)
            angle = np.round(np.deg2rad(np.arctan2(240 - 80, 160 - x) * 180 / np.pi - 90), 2)


            if abs(angle) > 70:

                return None

            return angle

        else:

            return None

    def circles(self, hsv, lower_color, upper_color):
        circles_list = []

        # Aplicar una mascara para el color especifico
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Encontrar contornos en la mascara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterar sobre los contornos encontrados
        for contour in contours:
            # Calcular el area y el perimetro del contorno
            new_contour = cv2.convexHull(contour)
            area = cv2.contourArea(new_contour)
            perimeter = cv2.arcLength(new_contour, True)

            if perimeter > 0:
                circularity = 4 * np.pi * area / perimeter

            if area > 300 and circularity > 0.5:
                circles_list.append(new_contour)
        return circles_list

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frameHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        red_circles = self.circles(frameHSV, self.lower_red, self.upper_red)
        green_circles = self.circles(frameHSV, self.lower_green, self.upper_green)
        yellow_circles = self.circles(frameHSV, self.lower_yellow, self.upper_yellow)

        angle_temp = self.line_following(self.image)

        if angle_temp:
            self.angle = angle_temp
            self.target.orientation = self.angle

        if red_circles:
            print("Circulo Rojo Detectado")
            self.target.id = 3

        elif yellow_circles:
            print("Circulo Amarillo Detectado")
            self.target.id = 2

        elif green_circles:
            print("Circulo Verde Detectado")
            self.target.id = 1

        else:
            print("Crculo no detectado")
            self.target.id = 0

        self.setpoint_pub.publish(self.target)

    def run(self):

        if self.image is None:
            print("Imagen no detectada")

        else:
            print("Procesando imagen")

if __name__ == '__main__':

    rospy.init_node("line_detection")

    image_proc = ImageProcessing()

    print("Nodo Inicializado")

    rate = rospy.Rate(10)  # Frecuencia de ejecucion de 2 Hz (ajustar segun sea necesario)

    while not rospy.is_shutdown():

        try:

            image_proc.run()
            rate.sleep()

        except rospy.ROSInterruptException:
            pass
