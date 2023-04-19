#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class Mobile_robot:

    def __init__(self, points, vel):

        self.linear_velocity = vel

        self.angular_left = vel * 4
        self.angular_rigth = vel * 2

        self.orientation = 0.0
        self.distance_traveled = 0.0

        self.wheel_radio = 0.05
        self.wheel_distance = 0.18

        self.route = np.array(points)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

        rospy.Subscriber("/wl", Float32, self.set_wr)
        rospy.Subscriber("/wr", Float32, self.set_wl)

        rospy.init_node("path_follower")
        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.stop)

    def set_wr(self, msg):
        self.angular_rigth = msg.data

    def set_wl(self, msg):
        self.angular_left = msg.data

    def stop(self):


        print("Recorrido finalizado en: {}".format(rospy.get_time()))
    
        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.velocity_pub.publish(msg)

    def get_target(pt_current, pt_obj):

        x, y = pt_current
        x2, y2 = pt_obj

        xx, yy = (x2-x, y2-y)

        distance = math.sqrt(xx*2 + yy*2)
        rotation = math.atan2(xx, yy)

        return (distance, rotation)


    def run(self):

        route_size = len(self.route)

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        last_rot = 0.0

        self.current_x = 0
        self.current_y = 0

        distance = 0
        rotation = 0

        current_time = rospy.get_time()
        last_time = rospy.get_time()

        actual_id = 0
        target_id = 1

        while not rospy.is_shutdown():

            current_time = rospy.get_time()
            dt = current_time - last_time
            last_time = current_time

            if actual_id != target_id:

                x1, y1 = self.route[actual_id]
                x2, y2 = self.route[target_id]

                xx, yy = (x2 - x1, y2 - y1)

                distance = math.sqrt(xx**2 + yy**2)
                
                buffer = math.atan2(xx, yy)
                rotation = buffer - last_rot
                last_rot = buffer

                if (rotation < -math.pi):

                    rotation += 2 * math.pi 

                elif (rotation > math.pi):

                    rotation -= 2 * math.pi


                actual_id += 1

                print("El punto objetivo     es: {} {}".format(x2, y2))
                print("El angulo objetivo    es: {}".format(math.degrees(rotation)))
                print("La distancia objetivo es: {}".format(distance))
                print("")
                print("")

            if self.orientation > rotation + 0.01 or self.orientation < rotation - 0.01:

                self.orientation += float(self.wheel_radio * ((self.angular_rigth - self.angular_left) / self.wheel_distance) * dt)

                if (self.orientation < -math.pi):

                    self.orientation += 2*math.pi 

                elif (self.orientation > math.pi):

                    self.orientation -= 2*math.pi

                msg.angular.z = 0.1 
                msg.linear.x = 0.0

                self.velocity_pub.publish(msg)

            elif self.distance_traveled < distance:

                self.distance_traveled += float(self.wheel_radio * ((self.angular_rigth + self.angular_left) / 2) * dt)

                msg.angular.z = 0.0
                msg.linear.x = self.linear_velocity

                self.velocity_pub.publish(msg)

            else:

                target_id += 1

                self.distance_traveled = 0.0
                self.orientation = 0.0

                if target_id >= route_size:

                    self.stop()
                    rospy.signal_shutdown("Route Completed")
                    self.rate.sleep()



if __name__ == "__main__":

    try:

        n = int(input("Ingresa el numero de puntos que conforman la ruta a seguir: "))
        print("")

        points = np.zeros((n+1,), dtype = [('x', int), ('y', int)])

        for i in range(n):

            px = int(input("Ingresa las coordenadas del punto {} en x: ".format(i)))
            py = int(input("Ingresa las coordenadas del punto {} en y: ".format(i)))
            print("")

            points[i+1] = (px, py)


        linear_velocity = float(input("Ingresa la velocidad con la que va a funcionar el robot (0.1 y 1.0) "))
        print("")

        puzzlebot = Mobile_robot(points, linear_velocity)

        puzzlebot.run()

    except rospy.ROSInterruptException:

        None
