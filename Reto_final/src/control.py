#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from puzzlebot_project.msg import setpoint

# Clase de controlador
class Controller(object):

    def __init__(self):
        rospy.init_node("controller", anonymous=True)
        self.loop_rate = rospy.Rate(30)
        rospy.on_shutdown(self.stop)

        # Velocidad máxima lineal y angular
        self.max_linear_vel  = 0.75 #60
        self.max_angular_vel = 5.5 #70

        # Punto objetivo
        self.target = setpoint()
        self.target.orientation = 0.0
        self.target.vel_level   = 0

        # Velocidad actual
        self.vel = Twist()
        self.vel.linear.x  = 0.0
        self.vel.linear.y  = 0.0
        self.vel.linear.z  = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0

        # Suscripción al tema "/set_point" para recibir el punto objetivo
        rospy.Subscriber("/set_point", setpoint, self.set_point_callback)

        # Variables de error anterior
        self.last_error_vel = 0.0
        self.last_error_ang = 0.0

        # Publicador para el tema "/cmd_vel" para enviar las velocidades
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)

    # Control de velocidad
    def controlVel(self, error, kp=0.90, kd=0.30):

        p = kp * error
        d = kd * (error - self.last_error_vel)
        self.last_error_vel = error
        return np.clip(p + d, 0.0, 120.0) / 120.0

    # Control de orientación angular
    def controlAng(self, error, kp=0.90, kd=0.10):

        # Ajuste de los valores proporcionales y derivativos basado en el error
        kd = np.clip((abs(error)//10) * kd, 0.05, 0.4)
        kp = np.clip((abs(error)//90) * kp, 0.2, 1.2)

        p = kp * error
        d = kd * (error - self.last_error_ang)
        self.last_error_ang = error
        return np.clip(p + d, -70.0, 70.0) / 70.0

    # Función de devolución de llamada para el punto objetivo
    def set_point_callback(self, msg):
        self.target.vel_level   = msg.vel_level
        self.target.orientation = msg.orientation

        # Ajuste del nivel de velocidad basado en la orientación
        self.target.vel_level *= 1 - (abs(self.target.orientation)/130.0)

        # Cálculo de las velocidades angular y lineal
        self.vel.angular.z = self.max_angular_vel * self.controlAng(self.target.orientation)
        self.vel.linear.x  = self.max_linear_vel  * self.controlVel(self.target.vel_level)

    # Detener el robot
    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.velocity_pub.publish(self.vel)

    # Ejecutar el controlador
    def run(self):
        self.velocity_pub.publish(self.vel)
        #print(self.vel.angular.z, self.vel.linear.x)

    # Iniciar el controlador
    def start(self):

        try:

            while not rospy.is_shutdown():

                self.run()
                self.loop_rate.sleep()

        except rospy.ROSInterruptException:

            pass

if __name__ == "__main__":
    # Crear una instancia del controlador y ejecutarlo
    control = Controller()
    control.start()
