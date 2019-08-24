# ROScode
You find here the code associated with our research "Multi-object detection for robotic grasping based on Convolutional Neural Networks". 

This code works with the uArm swift pro ROS package on this link https://github.com/uArm-Developer/RosForSwiftAndSwiftPro

and the Tensorflow object detection API on this link https://github.com/tensorflow/models/tree/master/research/object_detection

You find the ROS code on The tf_object_detection_node.py file, it is responsible for sending the detected regions of the objects to our robot that will use to execute grasps.


## Running the first Node
Once you have the node built you can run it with "rosrun tf_object_detection tf_object_detection_node.py".
Running this node alone will only appear the camera window detecting the objects. Having this working means that this detected regions are ready to be sent to the motion node.
## Node Information
Topics:

* `camera/image/raw`:  
  Subscribes `sensor_msgs/Image` Image which is used to show the realtime detection that will be sent to the robot.

* `tf_object_detection_node/start`:  
  Subscribes `std_msgs/Empty` Message to command object detection is carried out on the next image

* `tf_object_detection_node/adjusted_image`:  
  Publishes `sensor_msgs/Image` Adjusted image which may contain bounding boxes and the category classes of the detected objects
  
* `tf_object_detection_node/result`:  
  Publishes `tf_object_detection/detection_results` Contains an array of strings of the detected object names
  Publishes `tf_object_detection/rectangles_results` Contains an array of values of the bounding rectangles.
  
Parameters:

* `/object_detection/path`: the path for models/research/object_detection directory. Default value = /home/ubuntu/git/models/research/object_detection
* `/object_detection/confidence_level`: confidence level(class score), any object with a level below this will not be used. Default value = 0.7 (70%)

## Running the Second Node (move_group_python_uarmswiftpro.py)

Running this node allows to take the predicted regions with their category class from the first node to the uArm. This node works with Moveit messages. It basically has the functions necessary to execute the motion given a detected region on the objects. 

you can run it with "rosrun tf_object_detection move_group_python_uarmswiftpro.py".

## Node Information 

* '/move_group/display_planned_path'  is to create a `DisplayTrajectory`_ ROS publisher which is used to display
trajectories.

we will import the `moveit_commander`_ namespace. This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class, and a `RobotCommander`_ class. 

Because Moveit has a generic architecture that makes it able to control any Arm that was developed though it, We really recommend to refer to this Tutorial https://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html in case there are any issues with this particular node. the link includes many tutorials developed on both Python, and C++ that makes you able to choose your best language from them while working with moveit.


## Using the inference graph

The inference graph used in this demonstration is taken from our trained model that we applied on some specific objects that we advice not to take in your model. Better to train it using your own data. We describe the process of data collection in the paper. 

An easy way without training our same model, is to try one of the ZOO models like(SSD-mobilenet-COCO2018), it has been trained to detect 90 category classes, and can select some objects from them that you want to grasp. The problem with taking this model is that the detector doesn't detect the best grasping regions. The detected bounding rectangles in this case bound the whole object, so you want to do some little changes for the ROS code to make it successful with the grasping attempts. 

For more details on the ROS algorithm and the object detection model, please reach to our paper. and for any issues with code, kindly feel free to describe it on this link https://github.com/nouaim/uArmswfitpro-objectdetection/issues 
