import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np


class ObjectDetector(Node):

    def __init__(self):

        super().__init__('object_detector')

        self.pub = self.create_publisher(Point, '/object_center', 10)

        self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(0.03, self.loop)

        # HSV values from your calibration
        self.lower = np.array([100,123,66])
        self.upper = np.array([112,184,90])

    def loop(self):

        ret, frame = self.cap.read()

        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower, self.upper)

        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            return

        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)

        if M["m00"] == 0:
            return

        cx = int(M["m10"]/M["m00"])
        cy = int(M["m01"]/M["m00"])

        msg = Point()

        msg.x = float(cx)
        msg.y = float(cy)
        msg.z = 0.0

        self.pub.publish(msg)

        cv2.circle(frame,(cx,cy),6,(0,255,0),-1)

        cv2.imshow("camera",frame)
        cv2.waitKey(1)


def main():

    rclpy.init()

    node = ObjectDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
