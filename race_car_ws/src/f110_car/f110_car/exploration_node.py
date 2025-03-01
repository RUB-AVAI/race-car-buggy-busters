#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node

import numpy as np
import numpy.typing as npt

import math

from scipy.ndimage import label
from scipy.spatial.transform import Rotation

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon as Poly

from avai_lab import enums, utils

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Polygon, Point32
from racecar_msgs.msg import SemanticGrid

class ExplorationNode(Node):
    """
    Node that subscribes to the "/drive" topic, collects all AckermannDriveStamped msgs and
    converts them to Twist msgs which are published to the "/cmd_vel" topic.
    """
    def __init__(self):
        super().__init__("exploration_node") # "NodeName" will be displayed in rqt_graph
        self.declare_parameter("map_pose_topic", "/pose")
        self.declare_parameter("semantic_grid_topic", "/semantic_map")
        self.declare_parameter("target_point_topic", "/target_point")
        self.declare_parameter("outer_cones_topic", "/outer_cones")
        self.declare_parameter("inner_cones_topic", "/inner_cones")
        self.declare_parameter("projection_point_distance", 2)

        self.map_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("map_pose_topic").value,
                                                         self.pose_callback, 10)
        self.semantic_grid_subscriber = self.create_subscription(SemanticGrid, self.get_parameter("semantic_grid_topic").value, 
                                                                 self.semantic_grid_callback, 10)
        self.target_point_publisher = self.create_publisher(PoseStamped, self.get_parameter("target_point_topic").value, 10)
        self.outer_cones_publisher = self.create_publisher(Polygon, self.get_parameter("outer_cones_topic").value, 10)
        self.inner_cones_publisher = self.create_publisher(Polygon, self.get_parameter("inner_cones_topic").value, 10)
        self.forward_unit_vec = [1, 0, 0] # TODO: This will be wrong in the ROS2 coordinate system
        self.start_point = None
        self.last_pose = None
        self.exploration_points = []
        self.start_zone_left = False
        self.start_zone_reentered = False
        self.track_calculation = False
        self.outer_cones = None
        self.inner_cones = None

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if msg is None:
            self.get_logger().info("No pose received, skipping callback")
            return
        current_point = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        if self.start_point is not None:
            self.start_point = current_point
        self.exploration_points.append(current_point)
        self.last_pose = msg
        if (not self.start_zone_left) and (np.linalg.norm(utils.get_direction_vec(current_point, self.start_point)) > 0.5):
            self.start_zone_left = True
        elif self.start_zone_left and (np.linalg.norm(utils.get_direction_vec(current_point, self.start_point)) < 0.25):
            self.start_zone_reentered = True

    @staticmethod
    def semantic_grid_to_np(grid: SemanticGrid) -> npt.NDArray:
        width = grid.info.width
        height = grid.info.height
        labels = np.empty((height, width), dtype=int)
        for i, cell in enumerate(grid.cells):
            row = i // width
            col = i % width
            labels[row, col] = cell.label
        return labels

    def get_car_projection_vector(self) -> List:
        """Extract the current rotation from the last received pose message and create
        a vector pointing in the direction of the vehicle
        """
        assert self.last_pose is not None, "Pose not initialized"
        rotation_radians = utils.quat_to_rot_vec(self.last_pose.pose.pose.orientation.z, self.last_pose.pose.pose.orientation.w)
        rotation_axis = np.array([0, 0, 1])
         
        rotation_vector = rotation_radians * rotation_axis
        rotation = Rotation.from_rotvec(rotation_vector)
        return rotation.apply(self.forward_unit_vec)[:2]

    def get_projected_point(self) -> npt.NDArray:
        """Return a projected point in front of the vehicle that is in line with the current vehicle orientation"""
        assert self.last_pose is not None, "Pose not initialized"
        scaler = self.get_parameter("projection_point_distance").value
        projection_vector = self.get_car_projection_vector() * scaler
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        return projection_vector + vehicle_location

    def create_target_point_msg(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        t = self.get_clock().now()
        msg.header.stamp = t.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg
    
    def create_polygon_msg(self, points: np.ndarray) -> Polygon:
        msg = Polygon()
        msg_points = []
        for point in points:
            msg_point = Point32()
            msg_point.x = np.float32(point[0])
            msg_point.y = np.float32(point[1])
            msg_points.append(msg_point)
        msg.points = msg_points
        return msg

    def get_left_cone(self, cone_positions: npt.NDArray, labels: npt.NDArray, 
                       projected_point: npt.NDArray) -> npt.NDArray | None:
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        for cone_pos in cone_positions[labels == enums.YELLOW_CONE]:
            if utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        
        for cone_pos in cone_positions[labels == enums.UNKNOWN_CONE]:
            if utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        return None


    def get_right_cone(self, cone_positions: npt.NDArray, labels: npt.NDArray, 
                       projected_point: npt.NDArray) -> npt.NDArray | None:
        vehicle_location = np.array([self.last_pose.pose.pose.position.x, self.last_pose.pose.pose.position.y])
        for cone_pos in cone_positions[labels == enums.BLUE_CONE]:
            if not utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        
        for cone_pos in cone_positions[labels == enums.UNKNOWN_CONE]:
            if not utils.is_right(vehicle_location, projected_point, cone_pos):
                return cone_pos
        return None

    def resample_polygon(xy: np.ndarray, n_points: int = 100) -> np.ndarray:
        # credit to Tankred: https://stackoverflow.com/questions/66833406/resample-polygon-coordinates-to-space-them-evenly
        # Cumulative Euclidean distance between successive polygon points.
        # This will be the "x" for interpolation
        d = np.cumsum(np.r_[0, np.sqrt((np.diff(xy, axis=0) ** 2).sum(axis=1))])

        # get linearly spaced points along the cumulative Euclidean distance
        d_sampled = np.linspace(0, d.max(), n_points)

        # interpolate x and y coordinates
        xy_interp = np.c_[
            np.interp(d_sampled, d, xy[:, 0]),
            np.interp(d_sampled, d, xy[:, 1]),
        ]
        return xy_interp

    def sort_points(self, points: np.ndarray) -> np.ndarray:
        """
        Calculates the centroid of the points and sorts them by polar angle
        """
        # calculate centroid
        centroid = (sum([point[0] for point in points]) / len(points), sum([point[1] for point in points] / len(points)))
        # sort by polar angle
        points.sort(key = lambda point: math.atan2(point[1] - centroid[1], point[0] - centroid [0]))
        return points

    def calculate_track_polygon(self, cone_positions: np.ndarray):
        track_points = np.array(self.exploration_points)
        # needs parameter tuning for number of samples!
        track_samples = self.resample_polygon(track_points)
        sample_polygon = Polygon(track_samples)
        # check and sort cone positions based on whether they are inside or outside the sample_polygon
        outer_cones = []
        inner_cones = []
        for pos in cone_positions:
            point = Point(pos[0], pos[1])
            if sample_polygon.contains(point):
                inner_cones.append(pos)
            else:
                outer_cones.append(pos)
        sorted_outer_cones = self.sort_points(np.array(outer_cones))
        sorted_inner_cones = self.sort_points(np.array(inner_cones))
        return sorted_outer_cones, sorted_inner_cones


    def semantic_grid_callback(self, msg: SemanticGrid):
        if self.track_calculation:
            self.get_logger.info("Track polygon will be calculated, skipping semantic grid callback")
            if self.outer_cones and self.inner_cones:
                self.outer_cones_publisher.publish(self.create_polygon_msg(self.outer_cones))
                self.inner_cones_publisher.publish(self.create_polygon_msg(self.inner_cones))
            return
        if self.last_pose is None:
            self.get_logger().info("Pose not initialized, skipping semantic grid callback")
            return
        grid = self.semantic_grid_to_np(msg)
        valid_labels = [
            enums.YELLOW_CONE,
            enums.BLUE_CONE,
            enums.ORANGE_CONE,
            enums.UNKNOWN_CONE
        ]
        labels = grid
        cone_positions = []
        position_labels = []
        for cone_label in valid_labels:
            # Create a binary mask for this label.
            mask = (labels == cone_label)
            # Find connected clusters in the binary mask.
            labeled_mask, num_clusters = label(mask)
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            resolution = msg.info.resolution
            
            for cluster in range(1, num_clusters + 1):
                # Get indices (rows, cols) for cells belonging to this cluster.
                indices = np.where(labeled_mask == cluster)
                if len(indices[0]) == 0:
                    continue
                # Compute the centroid in grid coordinates.
                centroid_row = np.mean(indices[0])
                centroid_col = np.mean(indices[1])
                # Convert grid coordinates to world coordinates.
                centroid_x = origin_x + (centroid_col + 0.5) * resolution
                centroid_y = origin_y + (centroid_row + 0.5) * resolution
                cone_positions.append((centroid_x, centroid_y))
                position_labels.append(cone_label)
        position_labels = np.array(position_labels)
        projected_point = self.get_projected_point()
        cone_positions = np.array(cone_positions)
        if len(cone_positions) == 0:
            self.get_logger().info("No cones detected, skipping target point creation")
            return
        # calculate the track polygon if starting point is reapproached
        if self.start_zone_reentered:
            self.target_point_publisher.publish(self.create_target_point_msg(self.start_point[0], self.start_point[1]))
            self.outer_cones, self.inner_cones = self.calculate_track_polygon(cone_positions)
            self.get_logger().info("Start zone reentered, start calculating track polygon")
            self.track_calculation = True
            return
        # calculate the distances between the projected point and all cone positions
        distances = np.linalg.norm(cone_positions - projected_point, axis=1)
        # get the sorting index by distance
        dist_sort_idx = np.argsort(distances)
        sorted_labels = position_labels[dist_sort_idx]
        # Find the closest yellow cone that is to the right of the projection vector
        right_cone_position = self.get_right_cone(cone_positions[dist_sort_idx], sorted_labels, projected_point)
        if right_cone_position is None:
            self.get_logger().info("Could not locate a yellow cone to the right of the vehicle")
            return
        left_cone_position = self.get_left_cone(cone_positions[dist_sort_idx], sorted_labels, projected_point)
        if left_cone_position is None:
            self.get_logger().info("Could not locate a blue cone to the left of the vehicle")
            return
        target_point = right_cone_position + (left_cone_position - right_cone_position) / 2
        self.target_point_publisher.publish(self.create_target_point_msg(*target_point))

def main(args=None):
    rclpy.init(args=args)

    node = ExplorationNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()
