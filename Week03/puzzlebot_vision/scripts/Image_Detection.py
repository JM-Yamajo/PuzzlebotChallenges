#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class image_processing:
   
   def __init__(self):

      self.image = None
      
      self.bridge = cv_bridge.CvBridge()

      #Rangos de valores HSV
      self.lower_red = np.array([0, 100, 100])
      self.upper_red = np.array([10, 255, 255])
      self.lower_yellow = np.array([15, 100, 100])
      self.upper_yellow = np.array([35, 255, 255])
      self.lower_green = np.array([40, 100, 100])
      self.upper_green = np.array([80, 255, 255])

      #Subscribers y Publishers
      self.image_sub = rospy.Subscriber('/video_source/raw',Image, self.image_callback)
      self.flag_pub = rospy.Publisher('/color_Flags', String, queue_size=10)
      self.image_pub = rospy.Publisher('/proc_image', Image, queue_size=10)

   def image_callback(self,msg):
      print("Nueva imagen recivida")
      self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding = 'bgr8')


   def circles(self,image, lower_color, upper_color):

      circle = []

      # Convertir el frame a HSV
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

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
         circularity = 4 * np.pi * area / max(perimeter, 0.1)
         
         if area > 300 and circularity > 1.5 : 
               circle.append(new_contour)

      return circle
   
   def processing(self):

      frameHSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        
      for i in self.circles(self.image, self.lower_red, self.upper_red):

         print("Circulo Rojo Detectado")

         cv2.drawContours(self.image, [i], 0, (0,0,255),3)
         self.image_pub.publish(self.image)
         self.flag_pub.publish("R")


      for i in self.circles(self.image, self.lower_yellow, self.upper_yellow):

         print("Circulo Amarillo Detectado")

         cv2.drawContours(self.image, [i], 0, (0,255,255),3)
         self.image_pub.publish(self.image) 
         self.flag_pub.publish("Y")
         
      for i in self.circles(self.image, self.lower_green, self.upper_green):

         print("Circulo Verde Detectado")

         cv2.drawContours(self.image, [i], 0, (0,255,0),3)
         self.image_pub.publish(self.image)
         self.flag_pub.publish("G")

   def run(self):

      if self.image is None:
         
         print("Imagen no detectada")

         return
      
      self.processing()

         
if __name__ == '__main__':

   rospy.init_node("Image_Detection", anonymous = True)

   image_proc = image_processing()

   loop_rate = rospy.Rate(10)


   print("Nodo Inicializado")

   while not rospy.is_shutdown():

      try:

         image_proc.run()
         loop_rate.sleep()

      except rospy.ROSInterruptException:

         pass
