#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

# for the pick and place message
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Exercise-2 TODOs:

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005 # 0.005 seemed to work well after experimenting with several values
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter - table top and objects
    passthrough_z = cloud_filtered.make_passthrough_filter()
    passthrough_z.set_filter_field_name('z')
    passthrough_z.set_filter_limits(0.6, 1.1)
    cloud_filtered = passthrough_z.filter()

    # passthrough along x to remove table front
    passthrough_x = cloud_filtered.make_passthrough_filter()
    passthrough_x.set_filter_field_name('x')
    passthrough_x.set_filter_limits(0.331, 1.0)
    cloud_filtered = passthrough_x.filter()

    # RANSAC Plane Segmentation to detect table top
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    # Max distance for a point to be considered fitting the model
    max_distance = 0.01 # 0.1 from experimentation
    seg.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)

    # Extract outliers
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)  # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)

    # values from experimentation
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(150)
    ec.set_MaxClusterSize(4000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(pcl_to_ros(cloud_objects))
    pcl_table_pub.publish(pcl_to_ros(cloud_table))
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        # publishing detected objects one at a time, could not get publishing array to work
        detected_objects_pub.publish(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects - published individually above, could not get this to work as an array
    #detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')
    world_num = rospy.get_param('test_scene_num')

    test_scene_num.data = world_num

    # create a dictionary of dropbox params
    dropbox = {}
    for i in range(0, len(dropbox_param)):
        dropbox[dropbox_param[i]['group']] = {'arm': dropbox_param[i]['name'], 'position': dropbox_param[i]['position']}

    # Parse parameters into individual variables

    # Rotate PR2 in place to capture side tables for the collision map
    # world_joint_pub.publish(np.pi/2)

    # Loop through the pick list
    dict_list = []
    labels = []
    centroids = []  # to be list of tuples (x, y, z)
    for i in range(0, len(object_list_param)):
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        for object in object_list:
            if object.label == object_name.data:
                labels.append(object.label)

                # Get the PointCloud for a given object and obtain it's centroid
                points_arr = ros_to_pcl(object.cloud).to_array()
                centroid = np.mean(points_arr, axis=0)[:3]
                centroids.append(centroid)

                # Set 'pick_pose' for the object from centroid
                pick_pose.position.x = np.asscalar(centroid[0])
                pick_pose.position.y = np.asscalar(centroid[1])
                pick_pose.position.z = np.asscalar(centroid[2])

                # Create 'place_pose' for the object
                place_pose.position.x = dropbox[object_group]['position'][0]
                place_pose.position.y = dropbox[object_group]['position'][1]
                place_pose.position.z = dropbox[object_group]['position'][2]

                # Assign the arm to be used for pick_place
                arm_name.data = dropbox[object_group]['arm']

                # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
                # Populate various ROS messages
                yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
                dict_list.append(yaml_dict)

                # Wait for 'pick_place_routine' service to come up
                rospy.wait_for_service('pick_place_routine')

                try:
                    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                    # TODO: Insert your message variables to be sent as a service request
                    # resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)

                    # print ("Response: ",resp.success)

                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_' + str(world_num) + '.yaml', dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('pick_and_place', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObject, queue_size=1)

    world_joint_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=10)

    # Initialize color_list
    get_color_list.color_list = []

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
