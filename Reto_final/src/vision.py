#!/usr/bin/env python
import cv2
import rospy
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from puzzlebot_project.msg import centroids
from puzzlebot_project.msg import signal
from puzzlebot_project.msg import traffic_light

class Vision(object):
    def __init__(self):
        rospy.init_node("vision", anonymous=True)
        self.loop_rate = rospy.Rate(15)
        rospy.on_shutdown(self.stop)

        self.image = None
        self.bridge = cv_bridge.CvBridge()

        self.coefficients = centroids()
        self.coefficients.a = 0.0
        self.coefficients.b = 0.0
        self.coefficients.c = 0.0
        self.coefficients.cross = -1
        self.coefficients.empty = -1

        self.points = [640, 640, 640, 640, 640]

        self.signal = signal()
        self.signal.type = 0
        self.signal.area = 0

        self.light = traffic_light()
        self.light.red = 0.0
        self.light.yellow = 0.0
        self.light.green = 0.0
        self.light.area = 0.0

        """
        self.margins = margin() -> degrees
        self.margin.left = 0.0 
        self.margin.right = 0.0 
        self.margin.front_ang = 0.0
        self.margin.front_rise = 0.0
        """

        self.offset = 640
        self.centroid_pos = 640

        self.centroids = [[], [], [], [], []]

        rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        self.centroids_pub = rospy.Publisher("/centroids", centroids, queue_size=3)
        self.signals_pub = rospy.Publisher("/signals", signal, queue_size=3)
        self.lights_pub = rospy.Publisher("/traffic_lights", traffic_light, queue_size=3)

    def follower(self, centroids, default):
        lon = len(centroids)
        if 0 < lon < 4:
            close = min(centroids, key=lambda c: abs(c - default))
            if default-350 < close < default+350:
                return close, 1
        return default, np.clip(lon, 0, 2)

    def get_centroids(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for contour in contours:
            if cv2.contourArea(contour) > 450:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    #cY = int(M['m01'] / M['m00'])
                    centroids.append(cX)
        return centroids

    def line_follower(self, img):
        self.centroids[0] = self.get_centroids(img[648:719, 0:1279])
        self.centroids[1] = self.get_centroids(img[576:648, 0:1279])
        self.centroids[2] = self.get_centroids(img[503:576, 0:1279])
        self.centroids[3] = self.get_centroids(img[432:503, 0:1279])
        self.centroids[4] = self.get_centroids(img[360:432, 0:1279])

        lons = [1,1,1,1,1]

        self.points[0], lons[0] = self.follower(self.centroids[0], self.centroid_pos)
        self.points[1], lons[1] = self.follower(self.centroids[1], self.points[0])
        self.points[2], lons[2] = self.follower(self.centroids[2], self.points[1])
        self.points[3], lons[3] = self.follower(self.centroids[3], self.points[2])
        self.points[4], lons[4] = self.follower(self.centroids[4], self.points[3])

        self.coefficients.cross = lons.count(2)
        self.coefficients.empty = lons.count(0)

        self.centroid_pos = self.points[0]

        dominants = np.polyfit([760, 680, 600, 520, 440], self.points, 2)
        self.coefficients.a = dominants[0]
        self.coefficients.b = dominants[1]
        self.coefficients.c = dominants[2]

        #self.centroids_pub.publish(self.coefficients)
        #print(self.points)
        #print(lons)

    def segmentation(self, img):
        self.signal.type = 0
        self.signal.area = 0.0

        self.light.red = 0.0
        self.light.yellow = 0.0
        self.light.green = 1.0
        self.light.area = 500.0

        self.signals_pub.publish(self.signal)
        self.lights_pub.publish(self.light)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        self.line_follower(self.image)
        #self.segmentation(im_up)

    def stop(self):
        print("stopped")

    def run(self):
        self.centroids_pub.publish(self.coefficients)
        #print(self.points)

    def start(self):
        try:
            while not rospy.is_shutdown():
                self.run()
                self.loop_rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    node = Vision()
    node.start()
