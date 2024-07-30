#!/usr/bin/env python
import rospy
import numpy as np
from reto_final.msg import set_point
from reto_final.msg import motor_input
from reto_final.msg import motor_output

integral = 0
last_error = 0
v_max = 35
rt = 10
dt = 0.1

def pid_control(target_value, current_value):
    global integral, last_error

    error = target_value - current_value

    integral += error * dt
    integral = np.clip(integral, 0, 35) # limite la acumulacion
     
    derivative = (error - last_error)/0.1
    output = np.clip(((kp * error) + (ki * integral) + (kd * derivative))*A, 1, 35)
    last_error = error

    return output/35

def caract_motor(x):
    v6 =  0.00009367
    v5 = -0.00709
    v4 =  0.1954
    v3 = -2.28
    v2 =  11.7
    v1 =  10.14

    pwm = v6*pow(x, 5) + v5*pow(x, 4) + v4*pow(x, 3) + v3*pow(x, 2) + v2*x + v1
    return np.clip(pwm, 1, 255)/255

def setpoint_subscriber(msg):
    global motor_input_msg
    setpoint_msg.input = msg.input
    setpoint_msg.tm = msg.tm


def motor_output_subscriber(msg):
    global motor_output_msg
    motor_output_msg.output = msg.output
    motor_output_msg.tm = msg.tm
    motor_output_msg.st = msg.st

def get_params():
    global motor_input_msg, kp, ki, kd, is_pid, A, flag_tm, motor_input_msg, caract
    kp = rospy.get_param("controller_kp", 1)
    kd = rospy.get_param("controller_kd", 1)
    ki = rospy.get_param("controller_ki", 1)
    A = rospy.get_param("ganancia", 1)
    is_pid = rospy.get_param("pid", True) 
    caract = rospy.get_param("caract", False)

    motor_input_msg.tm = flag_tm*v_max or -v_max
    flag_tm = not flag_tm

    return True

if __name__=='__main__':
    global motor_input_msg, motor_output_msg, setpoint_msg, is_pid, flag_tm, caract
    motor_input_msg = motor_input()
    motor_output_msg = motor_output()
    setpoint_msg = set_point()
    flag_tm = True

    rospy.init_node("Controller")
    rate = rospy.Rate(rt)

    rospy.Subscriber("set_point", set_point, setpoint_subscriber)
    rospy.Subscriber("motor_output", motor_output, motor_output_subscriber)
    pub_2 = rospy.Publisher("motor_input", motor_input, queue_size = 1)

    while (not rospy.is_shutdown()) and get_params():
        direction = (setpoint_msg.input >= 0) or -1
        
        if (is_pid):
            motor_input_msg.input = pid_control(abs(setpoint_msg.input), abs(motor_output_msg.output))*direction
        else:
            if(caract):
                motor_input_msg.input = direction*caract_motor(abs(setpoint_msg.input))
            else:
                motor_input_msg.input = np.clip(setpoint_msg.input/v_max, -1, 1)

        pub_2.publish(motor_input_msg)
        rate.sleep()