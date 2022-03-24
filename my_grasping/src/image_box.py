#!/usr/bin/env python2.7

"""Get an image and command Spot to walk up to the selected object in the image."""

import rospy
import roslib
import time, sys
from sensor_msgs.msg import Image
from my_grasping.msg import PixelPose
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty


class image_display():

  def __init__(self):
    self.bridge = CvBridge()

    self.g_image_click = None
    self.g_image_display = None
    self.mU = True

    # Subscriber, Take only 1 image
    self.image_subscriber = rospy.Subscriber("/spot/camera/frontright/image", Image, self.image_callback, queue_size=10)


  def image_callback(self, img_msg):
    rospy.loginfo('Image received')
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
      rospy.logerr("CvBridge Error: {0}".format(e))

    # Flip image by 90 degrees
    self.cv_image = cv2.transpose(self.cv_image)
    self.cv_image = cv2.flip(self.cv_image, 1)

    # Show image to user and wait for them to click on a pixel
    rospy.loginfo('Click on an object to walk up to')
    image_title = 'Click to walk up to something'
    cv2.namedWindow(image_title)
    cv2.setMouseCallback(image_title, self.cv_mouse_callback)
    
    self.g_image_display = self.cv_image
    cv2.imshow(image_title, self.g_image_display)
    
    while self.g_image_click is None:
      key = cv2.waitKey(1) & 0xFF
      if key == ord('q') or key == ord('Q'):
        # Quit
        print('"q" pressed, exiting.')
        exit(0)

    rospy.loginfo('Walking to object at image location: ' + str(self.g_image_click[0]) + ' ' + str(self.g_image_click[1]))

    my_click = []
    my_click.append(self.g_image_click[0])
    my_click.append(self.g_image_click[1])
    
    # Publishing location of the object in the image
    while self.mU == True:
      print(my_click)
      self.location_publisher.publish(my_click)
      self.mU = False


  def cv_mouse_callback(self, event, x, y, flags, param):
    clone = self.g_image_display.copy()
    if event == cv2.EVENT_LBUTTONUP:
        self.g_image_click = (x, y)
    else:
        # Draw some lines on the image.
        # print('mouse', x, y)
        color = (30, 30, 30)
        thickness = 2
        image_title = 'Click to walk up to something'
        height = clone.shape[0]
        width = clone.shape[1]
        cv2.line(clone, (0, y), (width, y), color, thickness)
        cv2.line(clone, (x, 0), (x, height), color, thickness)
        cv2.imshow(image_title, clone)


  def main(self):
    rospy.init_node('image_display', anonymous=True)
    self.location_publisher = rospy.Publisher('object_location', PixelPose, queue_size=10)
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down ROS Image feature detector module"
      cv2.destroyAllWindows()

if __name__ == "__main__":
  imdis = image_display()
  imdis.main()
