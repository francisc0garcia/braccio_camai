#!/usr/bin/env python3

import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters
from edgetpu.detection.engine import DetectionEngine
from imutils.video import VideoStream
from PIL import Image
import argparse
import imutils
import time
import cv2

# Ros
import rospy
from sensor_msgs.msg import CompressedImage


class FaceDetectorEdgeTPU:

    def __init__(self):
        # Read input parameters
        self.input_image_compressed = rospy.get_param('~input_image_compressed', "usb_cam/image_raw/compressed")
        self.output_image_compressed = rospy.get_param('~output_image', "face_image/compressed")
        self.model_path = rospy.get_param('~model_path', "model.tflite")
        self.threshold = rospy.get_param('~threshold', 0.8)

        # print input parameters
        rospy.loginfo("input_image_compressed: " + self.input_image_compressed)
        rospy.loginfo("output_image_compressed: " + self.output_image_compressed)
        rospy.loginfo("model_path: " + self.model_path)
        rospy.loginfo("threshold: " + str(self.threshold))

        self.model = DetectionEngine(self.model_path)
        self.pub_image = rospy.Publisher(self.output_image_compressed, CompressedImage, queue_size=5)
        self.subscriber = rospy.Subscriber(self.input_image_compressed,  CompressedImage, self.callback, queue_size=5)

        rospy.spin()

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        orig = frame.copy()
        frame = Image.fromarray(frame)

        # make predictions on the input frame
        results = self.model.DetectWithImage(
            frame, threshold=self.threshold, keep_aspect_ratio=True, relative_coord=False)

        # loop over the results
        for r in results:
            # extract the bounding box and box and predicted class label
            box = r.bounding_box.flatten().astype("int")
            (startX, startY, endX, endY) = box

            # draw the bounding box and label on the image
            cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', orig)[1]).tostring()

        # Publish image with face detections
        self.pub_image.publish(msg)


def main(args):
    rospy.init_node('FaceDetectorEdgeTPU')

    FaceDetectorEdgeTPU()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
