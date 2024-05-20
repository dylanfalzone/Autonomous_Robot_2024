#! /usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse # we are creating a 'Trigger service'...
import warnings
import rospy
import numpy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import cv2 as cv2
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import itertools as it
import tf

# Publisher setup
pub_arm = rospy.Publisher('/arm_movement_data', String, queue_size = 10)
pub_drive = rospy.Publisher('/drive_data', String, queue_size = 10)
pub_rotate = rospy.Publisher('/rotate_data', Int16, queue_size = 10)
pub_done = rospy.Publisher('/done_commands', String, queue_size = 1)

bottle_within_frame = False
target = 'bottle'
depth_as_array = None
done = 'checking'
check = 0
Xmin = 0
Xmax = 0
Ymin = 0
Ymax = 0
Xmid = 0
focal_x = 537.7739387075931
focal_y = focal_x
c_x = 317.7413040471659
c_y = 224.2998926313842

def target_update(target_data):
    global target
    target = target_data.data

def depth_image_to_pointcloud(depth_image):

    u = numpy.tile(numpy.arange(depth_image.shape[1]), (depth_image.shape[0], 1))
    v = numpy.tile(numpy.arange(depth_image.shape[0]), (depth_image.shape[1], 1)).T

    x_over_z = (c_x - u) / focal_x
    y_over_z = (c_y - v) / focal_y

    z = depth_image / numpy.sqrt(1 + numpy.square(x_over_z) + numpy.square(y_over_z))
    x = x_over_z * z
    y = y_over_z * z

    point_cloud = numpy.zeros((depth_image.shape[0], depth_image.shape[1], 3))
    point_cloud[:, :, 0] = x
    point_cloud[:, :, 1] = y
    point_cloud[:, :, 2] = z

    return point_cloud


def rgb_box_data(bounding_boxes):
    global Xmin, Xmax, Ymin, Ymax, bottle_within_frame, Xmid, target
    bottle_within_frame = False
    Xmin = 0
    Xmax = 0
    Ymin = 0
    Ymax = 0
    Xmid = 0

    for ind, BB in enumerate(bounding_boxes.bounding_boxes):

        if BB.Class == target:
            Xmin = bounding_boxes.bounding_boxes[ind].xmin
            Xmax = bounding_boxes.bounding_boxes[ind].xmax 
            Ymin = bounding_boxes.bounding_boxes[ind].ymin
            Ymax = bounding_boxes.bounding_boxes[ind].ymax
            Xchange = (((Xmax - Xmin)/2) - 2)
            Ychange = (((Ymax - Ymin)/2) - 2)
            Xmin = Xmin + Xchange
            Xmax = Xmax - Xchange
            Ymin = Ymin + Ychange
            Ymax = Ymax - Ychange
            Xmid = Xmax - ((Xmax-Xmin)/2)

            if 270 < Xmid < 370:
                bottle_within_frame = True
            else:
                bottle_within_frame = False
            return True
    
            
def depth_data(head_camera):
    global depth_as_array 
    depth_as_array =  numpy.frombuffer(head_camera.data, numpy.uint16)
    depth_as_array.shape = ((480, 640))
                                                    # custom types
def trigger_response(request):
    global bottle_within_frame, Xmid, check

    rospy.loginfo("xmin: %s", str(Xmin))
    rospy.loginfo("xmax: %s", str(Xmax))
    rospy.loginfo("ymin: %s", str(Ymin)) 
    rospy.loginfo("ymax: %s", str(Ymax))

    distances = depth_image_to_pointcloud(depth_as_array)
    distances = distances[Ymin:Ymax, Xmin:Xmax].mean(axis=0).mean(axis=0)

    rospy.loginfo("Calculated distances: %s", str(distances))
    
    # Serialize distances and publish
    distances_str = " ".join(map(str, distances))
    temp = Int16()
    temp.data = int(Xmid)

    vector_distance = numpy.linalg.norm(distances)
    print(vector_distance) 

    if bottle_within_frame == False:
        check = 0
        pub_rotate.publish(temp)
    elif vector_distance > 850:
        check = 0
        pub_drive.publish(str(vector_distance))
    elif vector_distance < 750:
        check = 0
        pub_drive.publish(str(vector_distance))
    else:
        check += 1
        if check > 3:
            pub_arm.publish(distances_str)
            rospy.sleep(12)
            check = 0
        else:
            pub_done.publish(done)
    return TriggerResponse(
        success=True,
        message="service is up and running!"
    )

rospy.init_node('box_distances')
my_service = rospy.Service('/compute_distance', Trigger, trigger_response)
rospy.Subscriber('/head_camera/depth_registered/image', msg_Image, depth_data)
rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, rgb_box_data)
rospy.Subscriber('/target_data', String, target_update)
rospy.spin()                                      
