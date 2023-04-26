#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from puzzlebot_challenges.msg import SetPoint

class SetPointPublisher:

    def __init__(self, target_list):
        
        self.target_list = target_list
        self.pub = rospy.Publisher('/set_point', SetPoint, queue_size = 10)
        self.index = 0


def angle_wrap(target_thetha):

    if target_thetha > math.pi:

        target_thetha -= 2*math.pi

    elif target_thetha < -math.pi:

        target_thetha += 2*math.pi

    return target_thetha


# Función para solicitar al usuario las coordenadas goblales que conforman la trayectoria          
def get_target_list():

    current_x = 0.0
    current_y = 0.0

    n_points = int(input("Ingresa la cantidad de puntos que va tener la trayectoria: "))
    target_list = []

    prev_distance = 0.0

    for i in range(n_points):

        target_x = float(input(f"Ingresa la coordenda x del punto {i}: "))
        target_y = float(input(f"Ingresa la coordenda y del punto {i}: "))
        print("")
            
        target_distance = math.sqrt(((target_x - current_x) ** 2) + ((target_y - current_y) ** 2)) + prev_distance
        target_theta = math.atan2(target_y - current_y, target_x - current_x)
        target_theta = angle_wrap(target_theta)

        target_list.append((target_distance, target_theta))

        current_x = target_x
        current_y = target_y
        prev_distance = target_distance


    return target_list

def new_target_callback(msg):

    # Publica el siguiente punto en la lista si hay más elementos y el mensaje no está vacío
    if msg.data == "new_target":
        
        print("Nueva solicitud recibida")

        if publisher.index < len(publisher.target_list):

            target = SetPoint()
            
            target.id = publisher.index
            target.limit = len(publisher.target_list)
            target.distance, target.orientation = publisher.target_list[publisher.index]

            publisher.pub.publish(target)
            rospy.loginfo(f'Nuevo Objetivo Enviado: ({target.distance}, {target.orientation})')
            publisher.index += 1

        else:

            target = SetPoint()
            
            target.id = publisher.index
            target.limit = len(publisher.target_list)
            target.distance, target.orientation = publisher.target_list[publisher.index - 1]

            publisher.pub.publish(target)

            rospy.signal_shutdown('Trayectoria completada')

    elif msg.data == "continue":
        
        pass
                

if __name__ == '__main__':

    rospy.init_node("Set_Point", anonymous = True)

    target_list = get_target_list()

    try:

        publisher = SetPointPublisher(target_list)
        rospy.Subscriber("/status", String, new_target_callback)
        rospy.spin()

    except rospy.ROSInterruptException:

        rospy.loginfo('Nodo SetPoint Finalizado')
        pass
