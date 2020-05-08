#!/usr/bin/env python
from __future__ import print_function # TODO is this needed
import sys
import rospy
import copy
import roslib
import object_detection_lib
#from tf_object_detection.msg import rectangles_results
from std_msgs.msg import Empty
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import cv2


class ObjectDetectionNode(object):
    def __init__(self):
        self.bridge = CvBridge()

        # Publisher to publish update rectangles
        #self.__image_pub = rospy.Publisher("/tf_object_detection_node/send_a_box", rectangles_results, queue_size=1)


        # Publisher to publish update image
        self.__image_pub = rospy.Publisher("/tf_object_detection_node/adjusted_image", Image, queue_size=1)
        # Publisher to publish the result
        # Publisher ti publish bounding boxes
        



        # Subscribe to topic which will kick off object detection in the next image
        self.__command_sub = rospy.Subscriber("/tf_object_detection_node/start", Empty, self.StartCallback)
        # Subscribe to the topic which will supply the image fom the camera
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.Imagecallback)
        
        # Flag to indicate that we have been requested to use the next image
        self.__scan_next = False

        # Read the path for models/research/object_detection directory from the parameter server or use this default
        object_detection_path = rospy.get_param('/object_detection/path', '/home/nouaim/catkin_uarm/src/tf_object_detection/src')

        # Read the confidence level, any object with a level below this will not be used
        confidence_level = rospy.get_param('/object_detection/confidence_level', 0.50)

        # Create the object_detection_lib class instance
        self.__odc = object_detection_lib2.ObjectDetection(object_detection_path, confidence_level)

    # Callback for start command message
    def StartCallback(self, data):
        # Indicate to use the next image for the scan
        self.__scan_next = True

    # Callback for new image received
    def Imagecallback(self, data):

        #if self.__scan_next == True:
         #   self.__scan_next = False
            # Convert the ROS image to an OpenCV image
            try:
                  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                  print(e)
            # The supplied image will be modified if known objects are detected
            try: 
                 object_names_detected = self.__odc.scan_for_objects(cv_image)
            except  ODCError as r:
                 print(r) 

            #try: 
            #     object_boxes_detected = self.__odc.scan_for_boxes(cv_image)
            #except  ODCError as f:
            #     print(f)      
            
            # publish the image, it may have been modified

            # Publish names of objects detected
                     

            
                        
            

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
            print("the camera is working properly") 


            #try:
            #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            #except CvBridgeError as e:
            #    print(e)

                    
def main(args):
   

  
    print ("This is the camera detecting the object")
    un_object= ObjectDetectionNode()
    #un_object.callback()
    rospy.init_node('ObjectDetectionNode', anonymous=True)
    rospy.spin()
    #rate = rospy.Rate(0.3)
    cv2.destroyAllWindows()
    


    print ("============ Python tutorial demo complete!")
if __name__ == '__main__':
    main(sys.argv)
