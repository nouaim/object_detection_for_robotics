#!/usr/bin/env python
# Object Detection with TensorFlow and TensorFlow object detection API
# We will be using a pre-trained model from TensorFlow detection model zoo
# See https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

import numpy as np
import tensorflow as tf
from collections import defaultdict
import rospy

from io import StringIO
#from matplotlib import pyplot as plt
from PIL import Image
from utils import label_map_util
from utils import visualization_utils as vis_util
import os
from sensor_msgs.msg import Image, RegionOfInterest
from tf_object_detection.msg import detection_results, rectangles_results
from std_msgs.msg import Float32MultiArray



os.environ['TF_CPP_MIN_LOG_LEVEL'] = '12'
class ObjectDetection:

    def __init__(self, path, confidence): # path will be to the models/research/object_detection directory
        # Pre-trained model name
        #MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'
        PATH_TO_FROZEN_GRAPH = path +'/inference_graph'+ '/frozen_inference_graph.pb'
        PATH_TO_LABELS = path + '/data/' + 'labelmap.pbtxt'

        # Load a frozen Tensorflow model into memory
        self.__detection_graph = tf.Graph()

        with self.__detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # Open a session here. The first time we run the session it will take
            # a time to run as it autotunes, after that it will run faster
            self.__sess = tf.Session(graph=self.__detection_graph)

        # Load the label map. Label maps map indices to category names
        self.__category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

        # Store the confidence level
        self.__confidence = confidence

        #self.__pub = rospy.Publisher("/tf_object_detection_node/bresult", rectangles_results, queue_size=10)

        self.__result_pub = rospy.Publisher("/tf_object_detection_node/result2", detection_results, queue_size=10)

        


    def run_inference_for_single_image(self, image):
        with self.__detection_graph.as_default():
            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            tensor_dict = {}
            for key in [
                'num_detections', 'detection_boxes', 'detection_scores',
                'detection_classes', 'detection_masks'
            ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
            if 'detection_masks' in tensor_dict:
                # The following processing is only for single image
                detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                    detection_masks, detection_boxes, image.shape[0], image.shape[1])
                detection_masks_reframed = tf.cast(
                    tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                # Follow the convention by adding back the batch dimension
                tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)
            image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

            # Run inference
            output_dict = self.__sess.run(tensor_dict,feed_dict={image_tensor: np.expand_dims(image, 0)})

            # all outputs are float32 numpy arrays, so convert types as appropriate
            output_dict['num_detections'] = int(output_dict['num_detections'][0])
            output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
            output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
            output_dict['detection_scores'] = output_dict['detection_scores'][0]
            if 'detection_masks' in output_dict:
                output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict


    # This class function will be called from outside to scan the supplied img.
    # if objects are detected it will adjust the supplied image by drawing boxes areound the objects
    # The img parameter is an OpenCV image
    def scan_for_objects(self, image_np):
        # The img is already a numpy array of size height,width, 3

        # Actual detection.
        output_dict = self.run_inference_for_single_image(image_np)

        #print output_dict

        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.__category_index,
            instance_masks=output_dict.get('detection_masks'),
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=self.__confidence)

        # Return a list of object names detected
        detected_list = []
        recs=[[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0]]

        tab= [0,0,0,0,0,0,0,0,0,0,0,0]
        tab2= [0,0,0,0,0,0,0,0,0,0,0,0]
        
        k=0
      
        total_detections = output_dict['num_detections']
        print('total_detections= ',total_detections)
        result = detection_results()

        result.number_of_detections=total_detections
        
        o = output_dict['detection_boxes']

        if total_detections > 0:
            for detection in range(0, total_detections):
                if output_dict['detection_scores'][detection] > self.__confidence:
                    category = output_dict['detection_classes'][detection]
                    detected_list.insert(0,self.__category_index[category]['name'])
                    result.names_detected = detected_list
                    

            for detection in range(0, total_detections):
                for detectionj in range(0, 4):
                   if output_dict['detection_scores'][detection] > self.__confidence:
                       recs[detection][detectionj] = o [detection][detectionj]
                       

        #    for detection in range(0, total_detections):
        #        for detectionj in range(0, 4):
        #           if output_dict['detection_scores'][detection] > self.__confidence:
        #              tab[k]= recs[detection][detectionj]
        #              result.rectangles = tab
        #              k+=1                                         
                
                       
            print(detected_list)
            print(recs)           
            #print(boxes_results)

            #self.__pub.publish(boxes_results)
            
        #    self.__result_pub.publish(result)     
                          
             

        return tab2
   
        
