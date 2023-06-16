#!/usr/bin/env python
import rospy
import numpy as np
from puzzlebot_project.msg import setpoint
from puzzlebot_project.msg import centroids
from puzzlebot_project.msg import signal
from puzzlebot_project.msg import traffic_light

class Process(object):

    def __init__(self):
        rospy.init_node("decider", anonymous=True)
        self.loop_rate = rospy.Rate(15)
        rospy.on_shutdown(self.stop)

        self.target = setpoint()
        self.target.orientation = 0.0
        self.target.vel_level = 100

        self.coeff = [-6.830424097526668e-19, 1.5206532033710582e-15, 639.9999999999992]
        self.coeff_curv_l = [ 2.63871310e-03,  2.11350180e-01, -1.04474683e+03]
        self.coeff_curv_r = [-3.08541632e-03,  3.96166189e-01,  2.12105016e+03]

        self.ang = [0.0, 0.0, 0.0, 0.0]
        self.prev_cross = -1
        self.cross = 0

        rospy.Subscriber("/centroids", centroids, self.line_follower_callback)
        rospy.Subscriber("/signals", signal, self.signal_callback)
        rospy.Subscriber("/traffic_lights", traffic_light, self.traffic_light_callback)

        self.setpoint_pub = rospy.Publisher("/set_point", setpoint, queue_size=3)

    def traffic_light_callback(self, msg):
        pass

    def signal_callback(self, msg):
        pass

    def find_midpoint(self, coeff_1, coeff_2):
        a_avg = (coeff_1[0] + coeff_2[0]) / 2
        b_avg = (coeff_1[1] + coeff_2[1]) / 2
        c_avg = (coeff_1[2] + coeff_2[2]) / 2
        return a_avg, b_avg, c_avg

    def eval_model(self, coeff, xx):
        return coeff[0] * xx * xx + coeff[1] * xx + coeff[2]

    def line_follower_callback(self, msg):
        mcf = [msg.a, msg.b, msg.c]

        if msg.empty > 2:
            if msg.cross < 1:
                #self.output_model(self.coeff_curv_r)
                self.output_model(self.coeff)
                print("Recto")
            else:
                self.output_model(self.coeff_curv_l)
                print("Curvo")
        else:
            self.output_model(mcf)
            print("Model")


    def output_model(self, mcf): 
        model_1 = self.find_midpoint(self.coeff, mcf)
        model_2 = self.find_midpoint(model_1, mcf)
        model_3 = self.find_midpoint(model_2, mcf)
        model_4 = self.find_midpoint(model_3, mcf)

        x_1 = self.eval_model(self.coeff, 760)  # 95%
        x_2 = self.eval_model(model_1, 600)  # 75%
        x_3 = self.eval_model(model_2, 480)  # 60%
        x_4 = self.eval_model(model_3, 400)  # 50%
        x_5 = self.eval_model(model_4, 360)  # 45%

        #self.coeff = np.polyfit([760, 600, 480, 400], [x_1, x_2, x_3, x_4], 2)

        self.ang[0] = np.round(np.arctan2(160, x_2 - x_1) * 180 / np.pi - 90, 2)
        self.ang[1] = np.round(np.arctan2(120, x_3 - x_2) * 180 / np.pi - 90, 2)
        self.ang[2] = np.round(np.arctan2(80, x_4 - x_3) * 180 / np.pi - 90, 2)
        self.ang[3] = np.round(np.arctan2(40, x_5 - x_4) * 180 / np.pi - 90, 2)

    def stop(self):
        print("stopped")

    def run(self):
#        print("ang_1: ", self.ang[0])
#        print("ang_2: ", self.ang[1])
#        print("ang_3: ", self.ang[2])
#        print("ang_4: ", self.ang[3])
#        print()
        self.setpoint_pub.publish(self.target)
        self.target.orientation = self.ang.pop(0)
        self.ang.append(0.0)
        
    def start(self):
        try:
            while not rospy.is_shutdown():
                self.run()
                self.loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    node = Process()
    node.start()
