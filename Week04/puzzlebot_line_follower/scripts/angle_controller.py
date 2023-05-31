#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from puzzlebot_line_follower.msg import SetPoint
from puzzlebot_line_follower.msg import Location

class Mobile_Robot:

    def __init__(self):

        # Variables a usar
        self.shutdown = False

        self.orientation_error = 0.0

        self.prev_orientation_error = 0.0
        self.integral_orientation = 0.0
        self.derivative_orientation = 0.0

        self.start_time = rospy.get_time()
        self.last_time = rospy.get_time()
        self.current_time = rospy.get_time()

        # Parametros iniciales del Puzzlebot
        self.r = rospy.get_param("r", 0.05)
        self.l = rospy.get_param("l", 0.18)

        self.wl = rospy.get_param("wl", 0.0)
        self.wr = rospy.get_param("wr", 0.0)

        self.sample_time = rospy.get_param("sample_time", 0.125)

        self.max_linear_vel = rospy.get_param("max_linear_vel", 0.15)
        self.max_angular_vel = rospy.get_param("max_angular_vel", 0.25)

        self.current = Location()

        self.current.pos_x = rospy.get_param("pos_x", 0.0)
        self.current.pos_y = rospy.get_param("pos_y", 0.0)
        #self.current.vel = rospy.get_param("vel", 0.0)
        self.current.orientation = rospy.get_param("orientation", 0.0)

        # Parametros iniciales del controlador
        self.angular_kp = rospy.get_param("angular_kp", 0.75)
        self.angular_ki = rospy.get_param("angular_ki", 0.2)
        self.angular_kd = rospy.get_param("angular_kd", 0.075)

        # Mensaje de entrada
        self.target = SetPoint()

        self.target.orientation = 0.0
        self.target.id = 1

        # Mensaje de salida
        self.vel = Twist()

        self.vel.linear.x = rospy.get_param("linear_x", 0.0)
        self.vel.linear.y = rospy.get_param("linear_y", 0.0)
        self.vel.linear.z = rospy.get_param("linear_z", 0.0)

        self.vel.angular.x = rospy.get_param("angular_x", 0.0)
        self.vel.angular.y = rospy.get_param("angular_y", 0.0)
        self.vel.angular.z = rospy.get_param("angular_z", 0.0)

        # Subscribers
        rospy.Subscriber("/wr", Float32, self.wr_callback)
        rospy.Subscriber("/wl", Float32, self.wl_callback)
        rospy.Subscriber("/set_point", SetPoint, self.set_point_callback)

        # Publishers
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

    # Callback Functions
    def set_point_callback(self, msg):

        print("Nuevo Objetivo Recibido:")
        self.target.id = msg.id
        self.target.orientation = msg.orientation
        print(self.target.id, " --- ",self.target.orientation)
        print("")

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data
    
    def angular_control(self, id, dt):

        p = 0
        i = 0
        d = 0
        new_vel = 0

        # Calculo Control Proporcional
        p = self.angular_kp * self.orientation_error

        # Calculo control Integral
        self.integral_orientation += self.orientation_error * dt
        i = self.angular_ki * self.integral_orientation

        # Calculo Control Derivativo
        self.derivative_orientation = (self.orientation_error - self.prev_orientation_error) / dt
        d = self.angular_kd * self.derivative_orientation

        self.vel.linear.x = 0.0

        new_vel = p + i + d

        if id == "wl":

            if new_vel >= self.max_angular_vel:

                self.vel.angular.z = -self.max_angular_vel

            else:

                self.vel.angular.z = -new_vel

        else:

            if new_vel >= self.max_angular_vel:

                self.vel.angular.z = self.max_angular_vel

            else:

                self.vel.angular.z = new_vel

        # --- Semaforos ---
        if self.target.id == 1:

            self.vel.angular.z *= 1.0

        elif self.target.id == 2:
                
            self.vel.angular.z *= 0.5
      
        elif self.target.id == 3:

            self.vel.angular.z *= 0.0

        else:

            self.vel.angular.z *= 0.33

        self.vel.linear.x = 0.0
        self.velocity_pub.publish(self.vel)

    def angle_wrap(self, target_thetha):

        if target_thetha > math.pi:

            target_thetha -= 2*math.pi

        elif target_thetha < -math.pi:

            target_thetha += 2*math.pi

        return target_thetha

    def stop(self):

        print("Stop")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.velocity_pub.publish(self.vel)
    
    def run(self):
    
        if not self.shutdown:

            # Sample time
            self.current_time = rospy.get_time()
            dt = self.current_time - self.last_time

            # Calculo de error
            self.prev_orientation_error = self.orientation_error

            self.current.orientation = self.angle_wrap(self.current.orientation)

            self.orientation_error = self.target.orientation - self.current.orientation
            self.orientation_error = self.angle_wrap(self.orientation_error)

            print("Orientacion Objetivo:") 
            print(self.target.orientation)

        if self.orientation_error <= -0.02 or self.orientation_error >= 0.02:

            print("/// --- --- --- ///")

            print("Orientacion Actual:")
            print(self.current.orientation)
            print("")            

            print("Error De Orientacion:")
            print(self.orientation_error)
            print(" ")


            if self.orientation_error > 0.0:

                print("")
                print("Gira Izquierda")
                print("")

                self.angular_control("wl", dt)

                next_orientation = self.current.orientation - (self.r * ((self.wr - self.wl) / self.l) * dt)
                self.current.orientation = next_orientation

                self.last_time = rospy.get_time()

            elif self.orientation_error < 0.0:

                print("")
                print("Gira Derecha")
                print("")

                self.angular_control("wr", dt)

                next_orientation = self.current.orientation + (self.r * ((self.wr - self.wl) / self.l) * dt)
                self.current.orientation = next_orientation

                self.last_time = rospy.get_time()

        else:

            print("Trayectoria recta")

            if self.target.id == 1:

                self.vel.linear.x = self.max_linear_vel
        
            elif self.target.id == 2:
                
                self.vel.linear.x = self.max_linear_vel / 2.0
        
            elif self.target.id == 3:

                self.vel.linear.x = 0.0

            else:

                self.vel.linear.x = self.max_linear_vel / 3.0

            self.last_time = rospy.get_time()

            self.vel.angular.z = 0.0
            self.velocity_pub.publish(self.vel)

if __name__ == "__main__":

    # Inicializar el nodo
    rospy.init_node("angle_controller", anonymous = True)
    puzzlebot = Mobile_Robot()

    # Configurar el nodo
    loop_rate = rospy.Rate(20)
    rospy.on_shutdown(puzzlebot.stop)

    try:

        # Ejecutar el noodo
        while not rospy.is_shutdown():

            puzzlebot.run()

            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
            pass
