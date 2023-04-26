#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from puzzlebot_challenges.msg import SetPoint
from puzzlebot_challenges.msg import Location


class Mobile_Robot():

    def __init__(self):

        # Variables a usar
        self.shutdown = False
        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0

        self.distance_error = 0.0
        self.orientation_error = 0.0

        self.prev_distance_error = 0.0
        self.prev_orientation_error = 0.0

        self.integral_distance = 0.0
        self.integral_orientation = 0.0

        self.derivative_distance = 0.0
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
        self.traveled_distance = rospy.get_param("traveled_distance", 0.0)

        self.max_linear_vel = rospy.get_param("max_linear_vel", 0.12)
        self.max_angular_vel = rospy.get_param("max_angular_vel", 0.05)

        self.current = Location()

        self.current.pos_x = rospy.get_param("pos_x", 0.0)
        self.current.pos_y = rospy.get_param("pos_y", 0.0)
        self.current.orientation = rospy.get_param("orientation", 0.0)

        # Parametros iniciales del controlador
        self.linear_kp = rospy.get_param("linear_kp", 0.4)
        self.linear_ki = rospy.get_param("linear_ki", 0.2)
        self.linear_kd = rospy.get_param("linear_kd", 0.1)

        self.angular_kp = rospy.get_param("angular_kp", 0.3)
        self.angular_ki = rospy.get_param("angular_ki", 0.2)
        self.angular_kd = rospy.get_param("angular_kd", 0.1)

        # Mensaje de entrada
        self.target = SetPoint()

        self.target.id = 0
        self.target.limit = 1
        self.target.distance = 0.0
        self.target.orientation = 0.0

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
        self.status_pub = rospy.Publisher("/status", String, queue_size = 10)

    # Callback Functions
    def set_point_callback(self, msg):

        print("Nuevo Objetivo Recibido:")

        self.target.id = msg.id
        self.target.limit = msg.limit
        self.target.distance = msg.distance
        self.target.orientation = msg.orientation

        print(f"Target ID: {self.target.id}")
        print(f"Target Limit: {self.target.limit}")
        print(f"Target Distance: {self.target.distance}")
        print(f"Target Orientacion: {self.target.orientation}")
        print("--- --- --- ---")

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def control(self, id, dt):

        p = 0
        i = 0
        d = 0
        new_vel = 0

        if id == "l":

            print("Control Lineal")
            print("")

            # Calculo Control Proporcional
            p = self.linear_kp * self.distance_error

            # Calculo control Integral
            self.integral_distance += self.distance_error * dt
            i = self.linear_ki * self.integral_distance

            # Calculo Control Derivativo
            self.derivative_distance = (self.distance_error - self.prev_distance_error) / dt
            d = self.linear_kd * self.derivative_distance

            self.vel.angular.z = 0.0

            new_vel = p + i + d

            if new_vel >= self.max_linear_vel:

                self.vel.linear.x = self.max_linear_vel

            else:

                self.vel.linear.x = new_vel

        else:

            print("Control Angular")
            print("")

            # Calculo Control Proporcional
            p = self.angular_kp * self.orientation_error

            # Calculo control Integral
            self.integral_orientation += self.orientation_error * dt
            i = self.angular_ki * self.integral_orientation

            # Calculo Control Derivativo
            self.derivative_orientation = (
                self.orientation_error - self.prev_orientation_error) / dt
            d = self.angular_kd * self.derivative_orientation

            self.vel.linear.x = 0.0

            new_vel = p + i + d

            if id == "wl":

                if new_vel >= self.max_linear_vel:

                    self.vel.angular.z = -self.max_linear_vel

                else:

                    self.vel.angular.z = -new_vel

            else:

                if new_vel >= self.max_linear_vel:

                    self.vel.angular.z = self.max_linear_vel

                else:

                    self.vel.angular.z = new_vel

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
           
            print("Run")

            if self.target.id < self.target.limit:

                # Sample time
                self.current_time = rospy.get_time()
                dt = self.current_time - self.last_time

                # Calculo de error
                self.prev_distance_error = self.distance_error
                self.prev_orientation_error = self.orientation_error

                self.distance_error = self.target.distance - self.traveled_distance

                self.current.orientation = self.angle_wrap(self.current.orientation)

                self.orientation_error = self.target.orientation - self.current.orientation

                self.orientation_error = self.angle_wrap(self.orientation_error)

                print(f"--- Objetivo Actual: (distancia: {self.target.distance} , orientacion: {self.target.orientation})")

                print(f"dt: {dt}")

                # Simulacion del sistema dinamico
                if dt >= self.sample_time:
                
                    if self.orientation_error < -0.025 or self.orientation_error > 0.025:

                        print(f"Orientacion Actual: {self.current.orientation}")
                        print(f"Error De Orientacion: {self.orientation_error}")

                        print(" ")

                        print(f"wl Actual: {self.wl}")
                        print(f"wr Actual: {self.wr}")

                        if self.orientation_error > 0.0:

                            print("")
                            print("Gira Izquierda")
                            print("")

                            self.control("wl", dt)

                            print(f"Nuevo wl: {self.wl}")
                            print(f"Nuevo wr: {self.wr}")

                            next_orientation = self.current.orientation - (self.r * ((self.wr - self.wl) / self.l) * dt)
                            self.current.orientation = next_orientation

                            self.last_time = rospy.get_time()
                            self.status_pub.publish("continue")

                        elif self.orientation_error < 0.0:

                            print("")
                            print("Gira Derecha")
                            print("")

                            self.control("wr", dt)

                            print(f"Nuevo wl: {self.wl}")
                            print(f"Nuevo wr: {self.wr}")

                            next_orientation = self.current.orientation + (self.r * ((self.wr - self.wl) / self.l) * dt)
                            self.current.orientation = next_orientation

                            self.last_time = rospy.get_time()
                            self.status_pub.publish("continue")

                    elif self.distance_error > 0.025:

                        print(f"Distancia Recorrida: {self.traveled_distance}")
                        print(f"Error De Distancia: {self.distance_error}")

                        print(" ")

                        print(f"wl Actual: {self.wl}")
                        print(f"wr Actual: {self.wr}")

                        print("")
                        print("Avanzando")
                        print("")

                        self.control("l", dt)

                        print(f"Nuevo wl: {self.wl}")
                        print(f"Nuevo wr: {self.wr}")

                        dx = self.r * ((self.wr + self.wl) / 2.0) * dt * math.cos(self.current.orientation)
                        dy = self.r * ((self.wr + self.wl) / 2.0) * dt * math.sin(self.current.orientation)
                        dd = math.sqrt((dx ** 2) + (dy ** 2))

                        self.current.pos_x += dx
                        self.current.pos_y += dy
                        self.traveled_distance += dd

                        self.last_time = rospy.get_time()
                        self.status_pub.publish("continue")

                    else:

                        self.distance_error = 0.0
                        self.orientation_error = 0.0

                        self.last_time = rospy.get_time()
                        self.status_pub.publish("new_target")


            else:

                self.shutdown = True

        else:

            rospy.signal_shutdown("Trayectoria Finaliza")

if __name__ == '__main__':

    # Inicializar el nodo
    rospy.init_node("Controller", anonymous = True)
    puzzlebot = Mobile_Robot()

    # Configurar el nodo
    loop_rate = rospy.Rate(8)
    rospy.on_shutdown(puzzlebot.stop)

    try:

        # Correr el nodo
        while not rospy.is_shutdown():

            puzzlebot.run()

            print("")
            print("### ### ### ###")
            print("")

            loop_rate.sleep()

    except rospy.ROSInterruptException as e:
        rospy.logerr("An error occurred: %s", e)
