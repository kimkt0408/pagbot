#!/usr/bin/env python3

import rospy
import math
import tf
import tf2_ros
import tf2_py as tf2
import numpy as np
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
from scipy.optimize import minimize
from sklearn.cluster import DBSCAN
from std_msgs.msg import ColorRGBA

import actionlib

class PathFollower:
    def __init__(self):
        self.ROW_WIDTH = 0.76   # Width of the rows in the cornfields
        self.ROBOT_SIZE = 0.50

        self.LINEAR_VELOCITY = 0.3
        self.DISTANCE_THRESHOLD = 0.15
        self.ANGLE_THRESHOLD = 0.3

        self.MAX_HEIGHT = 1.0#-0.1
        self.MIN_HEIGHT = 0.3#-0.3
        self.NUM_POINTS_CLUSTER = 100

        self.NODE_INTERVAL = 0.5
        self.NODE_RADIUS = 2.0

        self.WAYPOINT_TYPE1 = "IN_ROW"
        self.WAYPOINT_TYPE2 = "NEXT_ROW"
        self.WAYPOINT_TYPE3 = "NODE"

        self.waypoint_idx = 0

        rospy.init_node('path_follower')
        rospy.Subscriber('/ns1/velodyne_points', PointCloud2, self.lidar_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path = []  # Store the path as a list of (x, y) coordinates
        self.rate = rospy.Rate(10)  # Publishing rate in Hz

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_listener = tf.TransformListener()

        self.waypoints = []  # List to store the selected waypoints
        self.waypoint_types = []

        self.quat_seq = []
        self.pose_seq = []

        self.init_node_prev = []

        self.lidar_callback_called = False


        self.waypoint_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.waypointCallback)
        self.waypoint2_sub = rospy.Subscriber('/clicked_point', PointStamped, self.waypointCallback2)

        # Create a MarkerArray message for visualization
        self.waypoint_marker_pub = rospy.Publisher('/waypoint_marker1', Marker, queue_size=10)
        self.node_marker_pub = rospy.Publisher('/node_marker', Marker, queue_size=10)
        self.line_pub = rospy.Publisher('/row_lines', Marker, queue_size=10)
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  
    # AMCL pose callback function
    def PoseListener(self):
        try:
            (self.trans_pose, self.rot_pose) = self.pose_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            #(self.trans_pose, self.rot_pose) = self.pose_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            self.pose_x, self.pose_y, _ = self.trans_pose
            _, _, self.pose_yaw = tf.transformations.euler_from_quaternion(self.rot_pose)
        
        except tf.Exception as e:
            rospy.logwarn("Could not get transform from /map to /base_link: %s" % e)
            #rospy.logwarn("Could not get transform from /odom to /base_link: %s" % e)


    def waypointCallback(self, data):
        # Extract the position and orientation from the received PoseStamped message
        position = data.pose.position
        orientation = data.pose.orientation

        waypoint = [position.x, position.y, position.z]
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.waypoints.append(waypoint)
        self.pose_seq.append(Pose(Point(*waypoint), Quaternion(*quaternion)))
        self.waypoint_types.append(self.WAYPOINT_TYPE1)

        for i, pose in enumerate(self.pose_seq):
            position = pose.position
            print(f"Waypoint {i+1}:")
            print(f"Position: ({position.x}, {position.y}, {position.z})")

        # Visualize waypoints
        waypoint_marker = self.waypointVisualization(self.waypoints, self.waypoint_types)
        
        # Publish the MarkerArray for visualization
        self.waypoint_marker_pub.publish(waypoint_marker)
        # # Save the waypoints to a YAML file
        # self.saveWaypointsToYaml()


    def waypointCallback2(self, clicked_point):
        # Process the received clicked point
        waypoint_x = clicked_point.point.x
        waypoint_y = clicked_point.point.y
        waypoint_z = clicked_point.point.z

        waypoint = [waypoint_x, waypoint_y, waypoint_z]

        rospy.loginfo("Received clicked point: x={}, y={}, z={}".format(waypoint_x, waypoint_y, waypoint_z))

        self.waypoints.append(waypoint)
        self.pose_seq.append(Pose(Point(*waypoint), Quaternion()))
        self.waypoint_types.append(self.WAYPOINT_TYPE2)

        # Visualize waypoints
        waypoint_marker = self.waypointVisualization(self.waypoints, self.waypoint_types)
        
        # Publish the MarkerArray for visualization
        self.waypoint_marker_pub.publish(waypoint_marker)


    def waypointVisualization(self, list, waypoint_types):
        marker = Marker()
        marker_id = 1               # ID counter for markers

        for i, element in enumerate(list):
            # Create a Marker message for each element
            p = Point()

            #marker.header.frame_id = "odom"  # Set the frame ID of the marker
            marker.header.frame_id = "map"  # Set the frame ID of the marker
            marker.id = marker_id  # Assign a unique ID to the marker
            marker.type = marker.SPHERE_LIST  # Use a sphere marker
            marker.action = marker.ADD  # Add the marker
            
            # Set the position of the marker
            p.x = element[0]
            p.y = element[1]
                        
            # Set the scale of the marker (adjust as needed)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # Set the color of the marker (adjust as needed)
            if waypoint_types[i] == 'IN_ROW':
                color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue color
            else:
                color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color
                
            # Append the marker to the MarkerArray
            marker.colors.append(color)
            marker.points.append(p)
        
        return marker

    # def nodeVisualization(self, list):
    #     marker = Marker()
    #     marker_id = 1               # ID counter for markers

    #     marker.header.frame_id = "odom"  # Set the frame ID of the marker
    #     marker.id = marker_id  # Assign a unique ID to the marker
    #     marker.type = marker.LINE_STRIP  # Use a sphere marker
    #     # marker.type = marker.SPHERE_LIST  # Use a sphere marker
                
    #     marker.scale.x = 0.05
    #     marker.scale.y = 0.05
    #     marker.scale.z = 0.05
    
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0
    
    #     for i, element in enumerate(list):
    #         # Create a Marker message for each element
    #         p = Point()
            
    #         # marker.type = marker.TEXT_VIEW_FACING  # Use a text marker
    #         marker.action = marker.ADD  # Add the marker
            
    #         # Set the position of the marker
    #         p.x = element[0]
    #         p.y = element[1]
    #         # p.z = element[2]
                
    #         # marker.text = str(marker_id)
    #         # marker_id += 1  # Increment the ID counter

    #         # Append the marker to the MarkerArray
    #         marker.points.append(p)
        
    #     return marker
    

    def nodeVisualization(self, list):
        marker = Marker()
        marker_id = 1               # ID counter for markers

        for i, element in enumerate(list):
            # Create a Marker message for each element
            p = Point()
            #marker.header.frame_id = "odom"  # Set the frame ID of the marker
            marker.header.frame_id = "map"  # Set the frame ID of the marker
            marker.id = marker_id  # Assign a unique ID to the marker
            marker.type = marker.LINE_STRIP  # Use a sphere marker
            marker.type = marker.SPHERE_LIST  # Use a sphere marker
            marker.action = marker.ADD  # Add the marker
            
            # Set the position of the marker
            p.x = element[0]
            p.y = element[1]
                        
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
        
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Append the marker to the MarkerArray
            marker.points.append(p)
        
        return marker
    

    def lineVisualization(self, list):
        lines_marker = Marker()

        #lines_marker.header.frame_id = "odom"
        lines_marker.header.frame_id = "map"
        lines_marker.type = lines_marker.LINE_LIST
        lines_marker.action = lines_marker.ADD
    
        lines_marker.scale.x = 0.03
        lines_marker.scale.y = 0.03
        lines_marker.scale.z = 0.03

        lines_marker.color.a = 1.0
        lines_marker.color.r = 1.0
        lines_marker.color.g = 1.0
        lines_marker.color.b = 0.0

        lines_marker.pose.orientation.x = 0.0
        lines_marker.pose.orientation.y = 0.0
        lines_marker.pose.orientation.z = 0.0
        lines_marker.pose.orientation.w = 1.0

        lines_marker.pose.position.x = 0.0
        lines_marker.pose.position.y = 0.0
        lines_marker.pose.position.z = 0.0


        for row in list:
            first_point = Point()
            second_point = Point()
            
            first_point.x = np.min(row[0])
            idx_min_x = np.argmin(row[0])
            first_point.y = row[1][idx_min_x]

            second_point.x = np.max(row[0])
            idx_max_x = np.argmax(row[0])
            second_point.y = row[1][idx_max_x]

            lines_marker.points.append(first_point)
            lines_marker.points.append(second_point)

        return lines_marker


    def lidar_callback(self, point_cloud):

        self.lidar_callback_called = False
        
        try:
            #trans = self.tf_buffer.lookup_transform("odom", point_cloud.header.frame_id, point_cloud.header.stamp, rospy.Duration(1.0))
            trans = self.tf_buffer.lookup_transform("odom", point_cloud.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            point_cloud_transformed = do_transform_cloud(point_cloud, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("Failed to transform point cloud: %s" % str(ex))
            return

        if len(self.pose_seq):
            self.PoseListener()

            # Process the transformed LiDAR data and generate a path with obstacle avoidance logic
            self.path = []
            self.path = self.generate_path(point_cloud_transformed)
            # print("PATH: ", self.path)
        
        self.lidar_callback_called = True
        rospy.sleep(1.0)
        
        
	#def lidar_callback(self, point_cloud):

     #   self.lidar_callback_called = False
     #  # Perform the frame transformation from lidar frame to map frame
     #   try:
     #       trans = self.tf_buffer.lookup_transform("odom", point_cloud.header.frame_id, point_cloud.header.stamp, rospy.Duration(10.0))
     #      point_cloud_transformed = do_transform_cloud(point_cloud, trans)
     #   except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
     #       rospy.logwarn("Failed to transform point cloud: %s" % str(ex))
     #       return

     #  if len(self.pose_seq):
     #       self.PoseListener()

     #       # Process the transformed LiDAR data and generate a path with obstacle avoidance logic
     #       self.path = []
     #       self.path = self.generate_path(point_cloud_transformed)
     #       # print("PATH: ", self.path)
        
     #   self.lidar_callback_called = True
     #   rospy.sleep(0.1)
        

    def generate_path(self, point_cloud):
        cloud_points = self.point_cloud_to_array(point_cloud)

        init_nodes = self.InitialNodes()

        if self.waypoint_types[self.waypoint_idx] == "IN_ROW":
            traversable_nodes = self.FindTraversableNode(cloud_points, init_nodes)
        else:
            traversable_nodes = init_nodes.copy()
        
        # Visualize waypoints
        node_marker = self.nodeVisualization(traversable_nodes)
        self.node_marker_pub.publish(node_marker)

        return traversable_nodes
    

    def point_cloud_to_array(self, point_cloud):    
        assert isinstance(point_cloud, PointCloud2)
        cloud_points = list(pc2.read_points(point_cloud, skip_nans=True, field_names=("x", "y", "z")))
        
        return cloud_points

    
    def InitialNodes(self): # MODIFY
        init_nodes = []
        
        nearest_goal_pose = self.pose_seq[0]
        nearest_goal_pose_x = nearest_goal_pose.position.x
        nearest_goal_pose_y = nearest_goal_pose.position.y

        # print("NEAREST: ", nearest_goal_pose_x, nearest_goal_pose_y, self.pose_seq[0])
        # Calculate the distance between the robot pose and the goal pose
        dx = nearest_goal_pose_x - self.pose_x
        dy = nearest_goal_pose_y - self.pose_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate the number of nodes needed based on the distance and interval
        num_nodes = int(distance / self.NODE_INTERVAL)
        
        if num_nodes == 0:
            init_nodes = self.init_node_prev.copy()
        else:
            # Calculate the incremental step for each coordinate
            step_x = dx / num_nodes
            step_y = dy / num_nodes
            
            # Generate the nodes along the line segment
            current_x = self.pose_x
            current_y = self.pose_y
            
            for _ in range(num_nodes):
                current_x += step_x
                current_y += step_y
                init_nodes.append((current_x, current_y))
            
            self.init_node_prev = init_nodes.copy()
        
        return init_nodes
    

    def distance(self, point1, point2):
        # Calculate the Euclidean distance between two points
        return np.linalg.norm(point1 - point2)


    def distance_to_points(self, point, points):
        # Calculate the distances between the point and each of the points
        distances = [self.distance(point, p) for p in points]
        if len(distances) > 0:
            min_distance = np.min(distances)
            min_index = np.argmin(distances)

            return min_distance, min_index
        else:
            return float('inf'), -1


    def project_point_to_line(self, point, line):
        x0, y0 = point
        m, b = line

        # Compute the perpendicular distance from the point to the line
        d = abs(y0 - m*x0 - b) / math.sqrt(m**2 + 1)

        # Compute the x-coordinate of the projected point
        x_proj = (m*y0 + x0 - m*b) / (m**2 + 1)

        # Compute the y-coordinate of the projected point
        y_proj = m*x_proj + b

        return x_proj, y_proj


    def FindTraversableNode(self, cloud_points, init_nodes):
        nearest_node = init_nodes[0]
        nearest_node_x = nearest_node[0]
        nearest_node_y = nearest_node[1]

        nearest_node_list = [nearest_node_x, nearest_node_y]
        nearest_node_arr = np.array(nearest_node_list)
        
        surrounding_points = [list(cloud_point) for cloud_point in cloud_points if self.distance(cloud_point[0:2], nearest_node_arr) <= self.NODE_RADIUS and cloud_point[2] < self.MAX_HEIGHT and cloud_point[2] > self.MIN_HEIGHT] 
        
        # (2) Clustering the pointclouds from each row
        if len(surrounding_points) > 0:
            clustering = DBSCAN(eps=0.3, min_samples=3).fit_predict(surrounding_points)
            
            num_clusters = max(clustering) + 1

            self.cluster_list = [[] for _ in range(num_clusters)]

            for i in range(len(surrounding_points)):
                if clustering[i] != -1:
                    self.cluster_list[clustering[i]].append(surrounding_points[i])

            row_list = []
            for cluster in self.cluster_list:
                if len(cluster) > self.NUM_POINTS_CLUSTER:
                    x = np.array([point[0] for point in cluster])
                    y = np.array([point[1] for point in cluster])

                    slope, intercept = np.polyfit(x, y, 1)
                    row_list.append([x, y, slope, intercept])

            lines_marker = self.lineVisualization(row_list)
            self.line_pub.publish(lines_marker)

            left_points_proj = []
            right_points_proj = []

            for row_line in row_list:
                x_proj, y_proj = self.project_point_to_line(nearest_node, [row_line[2], row_line[3]])
                
                vec_pose_2_node = np.array([(nearest_node_x-self.pose_x), (nearest_node_y-self.pose_y), 0])
                vec_pose_2_proj = np.array([(x_proj-self.pose_x), (y_proj-self.pose_y), 0])
                
                cross_product = np.cross(vec_pose_2_node, vec_pose_2_proj)

                if cross_product[2] > 0:    # left
                    left_points_proj.append([x_proj, y_proj])
                else:                       # right
                    right_points_proj.append([x_proj, y_proj])

            left_min_dist, left_min_idx = self.distance_to_points(np.array(nearest_node_list), np.array(left_points_proj))
            right_min_dist, right_min_idx = self.distance_to_points(np.array(nearest_node_list), np.array(right_points_proj))

            # If there is no clusters to left/right
            if left_min_dist == float('inf') and right_min_dist != float('inf'):
                new_node = nearest_node_list

            elif left_min_dist != float('inf') and right_min_dist == float('inf'):
                new_node = nearest_node_list

            elif left_min_dist == float('inf') and right_min_dist == float('inf'):
                new_node = nearest_node_list

            else:
                new_node = np.mean([left_points_proj[left_min_idx], right_points_proj[right_min_idx]], axis=0)

        else:
            new_node = nearest_node_list

        traversable_nodes = init_nodes.copy()
        traversable_nodes[0] = (new_node[0], new_node[1])

        # print("BEFORE OPTIMAL:", init_nodes[0])
        # print("AFTER OPTIMAL:", traversable_nodes[0])

        return traversable_nodes


    def follow_path(self):
        while not rospy.is_shutdown():
            if self.lidar_callback_called:
                if self.path:
                    target_x, target_y = self.path[0]
                    print("TARGET: ", target_x, target_y)

                    dist_x = target_x-self.pose_x
                    dist_y = target_y-self.pose_y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    goal_position_angle = math.atan2(dist_y, dist_x)
                    heading = self.pose_yaw

                    # print(goal_position_angle, heading)
                    angle_target_heading = goal_position_angle - heading

                    # print(angle_target_heading, goal_position_angle, heading)

                    while(1):
                        if (angle_target_heading < -math.pi):
                            angle_target_heading += 2*math.pi

                        elif (angle_target_heading > math.pi):
                            angle_target_heading -= 2*math.pi

                        else:
                            break                

                    velocity_msg = Twist()

                    if (abs(angle_target_heading) > math.pi*0.5) & (abs(angle_target_heading) < math.pi*1.5):
                        if angle_target_heading < 0:
                            angle_target_heading_tmp = angle_target_heading+math.pi
                        else:
                            angle_target_heading_tmp = angle_target_heading-math.pi  # Adjust this value as needed
                            
                        if abs(angle_target_heading_tmp) > self.ANGLE_THRESHOLD:
                            velocity_msg.angular.z = angle_target_heading_tmp
                            print("ROTATION (BACKWARD)")
                        else:
                            velocity_msg.linear.x = -self.LINEAR_VELOCITY 
                            print("MOVE (BACKWARD)")

                    else:
                        if abs(angle_target_heading) > self.ANGLE_THRESHOLD:
                            velocity_msg.angular.z = angle_target_heading  # Adjust this value as needed
                            print("ROTATION (FORWARD)")
                        else:
                            velocity_msg.linear.x = self.LINEAR_VELOCITY
                            print("MOVE (FORWARD)")
                        
                    self.velocity_publisher.publish(velocity_msg)
                    # print(velocity_msg)

                    # print("Distance/Angle: ", distance, "/", angle_target_heading)
                    
                    final_goal_pose = self.pose_seq[-1]

                    final_goal_pose_x = final_goal_pose.position.x
                    final_goal_pose_y = final_goal_pose.position.y


                    # Calculate the distance between the robot pose and the goal pose
                    final_dx = final_goal_pose_x - self.pose_x
                    final_dy = final_goal_pose_y - self.pose_y
                    final_distance = math.sqrt(final_dx**2 + final_dy**2)

                    # print(distance, final_distance, self.DISTANCE_THRESHOLD)
                    # print(distance < self.DISTANCE_THRESHOLD)
                    # print("WAYPOINTS: ", self.waypoints)

                    # print("REMAIN NODE: ", self.path)
                    print(self.path)
                    if distance < self.DISTANCE_THRESHOLD:
                        self.path.pop(0)
                        rospy.loginfo("Change to the next node!")
                        if not len(self.path):
                            self.pose_seq.pop(0)
                            self.waypoint_idx += 1
                            print(self.pose_seq)
                            rospy.loginfo("Change to the next goal pose!")
                
                    if final_distance < self.DISTANCE_THRESHOLD or not len(self.pose_seq):
                        # self.pose_seq.pop(0)
                        rospy.loginfo("Final goal pose reached!")
                        rospy.signal_shutdown("Final goal pose reached!")
                        
                        return
                        
                    print("=============================================")
                #self.lidar_callback_called = False
            else:
                self.rate.sleep()
                


if __name__ == '__main__':
    path_follower = PathFollower()
    path_follower.follow_path()
