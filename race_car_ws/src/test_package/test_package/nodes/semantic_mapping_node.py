#!/usr/bin/env python3

from collections import defaultdict
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from racecar_msgs.msg import DetectedConeArray, SemanticGrid, SemanticCell
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration
from scipy.ndimage import label, center_of_mass
from avai_lab.enums import UNKNOWN_CONE

class SemanticMappingNode(Node):
    """Node for building a semantic map by integrating cone detections into a SLAM map."""
    
    def __init__(self):
        super().__init__('semantic_mapping_node')
        
        # Declare parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cones_topic', '/yolo_cones')
        self.declare_parameter('semantic_grid_topic', '/semantic_map')
        self.declare_parameter('filtered_map_topic', '/filtered_map')
        self.declare_parameter('max_cone_length_m', 0.3) #Maximum length for an object to be detected in our filtered map
        self.declare_parameter('min_cone_area_m2', 0.001) #Minimum area for an object to be detected in our filtered map
        self.declare_parameter('cluster_merge_threshold', 0.1) # maximum distance (in meters) for a cone to “label” a nearby cluster, also used to identify identical clusters if they drift
        
        # Retrieve parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.cones_topic = self.get_parameter('cones_topic').value
        self.semantic_grid_topic = self.get_parameter('semantic_grid_topic').value
        self.filtered_map_topic = self.get_parameter('filtered_map_topic').value
        self.cluster_merge_threshold = self.get_parameter('cluster_merge_threshold').value
        self.max_cone_length_m = self.get_parameter('max_cone_length_m').value
        self.min_cone_area_m2 = self.get_parameter('min_cone_area_m2').value
        
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 1)
        self.cones_sub = self.create_subscription(DetectedConeArray, self.cones_topic, self.cones_callback, 1)
        
        self.filtered_map_pub = self.create_publisher(OccupancyGrid, self.filtered_map_topic, 1)
        self.semantic_pub = self.create_publisher(SemanticGrid, self.semantic_grid_topic, 1)
        
        # Variables to store the current map and known cone detections
        self.current_map = None
        self.new_cones = []  # List of tuples: (grid_x, grid_y, label)
        
        # Persistent cluster data:
        #   key: cluster_id (int)
        #   val: {
        #       "centroid": (cx, cy),       # in grid coordinates
        #       "label_counts": {label_val: hits, ...}
        #   }
        self.clusters_dict = {}
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Semantic Mapping Node started.")

    def map_callback(self, msg):
        """
        Callback for receiving the occupancy grid map.
        Filters the map & then builds a semantic map with the filtered map.
        
        :param msg: The occupancy grid message from SLAM Toolbox.
        :return: None
        """
        self.current_map = msg
        
        filtered_map = self.filter_large_objects(msg)
        self.filtered_map_pub.publish(filtered_map)
        
        semantic_msg = self.build_semantic_grid(filtered_map)
        self.semantic_pub.publish(semantic_msg)
        
        self.new_cones.clear()

    def cones_callback(self, msg):
        """
        Callback for receiving new cone detections.
        Transforms the cones into the map frame and converts to grid coordinates.
        
        :param msg: The DetectedConeArray message.
        :return: None
        """
        if not self.current_map:
            return
        map_frame = self.current_map.header.frame_id
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        resolution = self.current_map.info.resolution
        
        for cone in msg.cones:
            trans_point = self.transform_point_to_frame(cone.position, cone.header, map_frame)
            if trans_point is None:
                continue
            # Convert the cone’s world coordinates to grid coordinates.
            grid_x, grid_y = self.world_to_grid(trans_point.point.x, trans_point.point.y, origin_x, origin_y, resolution)
            self.new_cones.append((grid_x, grid_y, cone.type))

    def build_semantic_grid(self, filtered_map):
        """
        Build a semantic grid by overlaying detected cones onto clusters in the filtered map.
        For each connected cluster of occupied cells (value 100), if a cone detection is within
        a cluster_merge_threshold (converted to grid cells), the cluster gets a 'hit' for this label.
        All cells in a cluster are labeled with the label that has the highest amount of hits.
        Clusters and their label hits are persisted (and centroids updated if they drift a bit in the map).
        
        :param filtered_map: The filtered occupancy grid.
        :return: A SemanticGrid message with label information.
        """
        semantic_grid = SemanticGrid()
        semantic_grid.header.stamp = self.get_clock().now().to_msg()
        semantic_grid.header.frame_id = filtered_map.header.frame_id
        semantic_grid.info = filtered_map.info

        width = filtered_map.info.width
        height = filtered_map.info.height
        resolution = filtered_map.info.resolution

        #First, populate everything according to the filtered map
        data_in = list(filtered_map.data)
        semantic_cells = []
        for cell_val in data_in:
            scell = SemanticCell()
            scell.occupancy = cell_val
            if cell_val == 0:
                scell.label = 0
            elif cell_val == -1:
                scell.label = -1
            else:
                scell.label = UNKNOWN_CONE
            semantic_cells.append(scell)

        # Convert grid data into a 2D NumPy array
        data_array = np.array(filtered_map.data).reshape((height, width))
        
        # Create a mask where occupied cells are True and free/unknown cells are False
        occupied_mask = (data_array == 100)
        
        # Find/Label connected regions in the 2d array
        labeled_array, num_connected_regions = label(occupied_mask)
        
        #If no connected regions are found, skip labeling
        if num_connected_regions == 0 or not self.new_cones:
            semantic_grid.cells = semantic_cells
            return semantic_grid
        
        # Compute centroids (in grid coordinates) for all regions at once.
        # center_of_mass returns (row, col) for each label.
        centroids = center_of_mass(occupied_mask, labeled_array, 
                                index=range(1, num_connected_regions + 1))
        
        # Convert cone detections into a NumPy array for vectorized distance checks.
        cones_np = np.array(self.new_cones) # shape (N, 3) because columns are grid_x, grid_y, cone_label

        # Convert the cone-to-cluster threshold from meters to grid cells
        threshold_in_cells = self.cluster_merge_threshold / resolution

        # Build a mapping from temporary region numbers to persistent cluster IDs
        region2cluster = {}

        #Go through each region, match to stored clusters and update hits/cone detections
        for region_num, centroid in enumerate(centroids, start=1):
            # centroid is (row, col); use directly as grid coordinates.
            centroid_x_grid = centroid[1]
            centroid_y_grid = centroid[0]

            # Match to an existing cluster (or create a new one).
            cluster_id = self.match_cluster_to_existing(centroid_x_grid, centroid_y_grid, threshold_in_cells)
            if cluster_id is None:
                cluster_id = len(self.clusters_dict)
                self.clusters_dict[cluster_id] = {
                    "centroid": (centroid_x_grid, centroid_y_grid),
                    "label_counts": defaultdict(int)
                }
            else:
                # If the cluster already exists, update the centroid to account for minor drift.
                self.clusters_dict[cluster_id]["centroid"] = (centroid_x_grid, centroid_y_grid)
            region2cluster[region_num] = cluster_id

            # If there are cone detections, update the clusters label hits
            if cones_np.shape[0] > 0:
                # Compute distances from this centroid to all cone detections in grid cells
                distances = np.hypot(cones_np[:, 0] - centroid_x_grid,
                                     cones_np[:, 1] - centroid_y_grid)
                # Find cone detections within the threshold.
                hits = np.where(distances < threshold_in_cells)[0]
                #Update hits
                for idx in hits:
                    cone_label = int(cones_np[idx, 2])
                    self.clusters_dict[cluster_id]["label_counts"][cone_label] += 1

        # Now assign labels to each region in the semantic grid.
        for region_num in range(1, num_connected_regions + 1):
            #Get all cells form the current region/cluster
            region_indices = np.where(labeled_array == region_num)
            if len(region_indices[0]) == 0:
                continue
            cluster_id = region2cluster.get(region_num)
            label_counts = self.clusters_dict[cluster_id]["label_counts"]
            if not label_counts:
                continue
            best_label = int(max(label_counts, key=label_counts.get))

            # Label all cells in the cluster with the cone's best (most hit) label
            flat_indices = region_indices[0] * width + region_indices[1]
            for flat_idx in flat_indices:
                semantic_cells[flat_idx].label = best_label

        semantic_grid.cells = semantic_cells
        
        return semantic_grid

    
    def match_cluster_to_existing(self, centroid_x, centroid_y, threshold):
        """
        Given a cluster centroid in grid coordinates, try to match it with an existing cluster.
        Returns the cluster_id if a match is found (distance less than threshold), otherwise None.
        
        :param centroid_x: The x-coordinate in grid cells.
        :param centroid_y: The y-coordinate in grid cells.
        :param threshold: The merge threshold in grid cells.
        """
        min_dist = float('inf')
        best_cluster_id = None

        for cid, cluster_info in self.clusters_dict.items():
            cx, cy = cluster_info["centroid"]
            dist = np.hypot(centroid_x - cx, centroid_y - cy)
            if dist < threshold and dist < min_dist:
                min_dist = dist
                best_cluster_id = cid

        return best_cluster_id

    def filter_large_objects(self, occupancy_map):
        """
        Filter out objects from the occupancy map that are either too large in any dimension
        or too small in overall area. For each connected cluster of occupied cells, we compute
        the bounding box in meters and also compute the area in square meters. We remove clusters
        that exceed max_cone_length_m in width or height, or that have an area below min_cone_area_m2.
        
        :param occupancy_map: The raw OccupancyGrid.
        :return: A new OccupancyGrid with large clusters filtered.
        """
        width = occupancy_map.info.width
        height = occupancy_map.info.height
        resolution = occupancy_map.info.resolution
        # Convert grid data into a 2D NumPy array
        data_array = np.array(filtered_map.data).reshape((height, width))
        
        # Create a mask where occupied cells are True and free/unknown cells are False
        occupied_mask = (data_array == 100)
        
        # Find/Label connected regions in the 2d array
        labeled_array, num_connected_regions = label(occupied_mask)
        
        # Iterate over each connected region
        for region in range(1, num_connected_regions + 1):
            #Get all cells from the current region/cluster
            region_indices = np.where(labeled_array == region)
            if len(region_indices[0]) == 0:
                continue
            
            # Calculate the bounding box in grid coordinates.
            min_x = np.min(region_indices[1])
            max_x = np.max(region_indices[1])
            min_y = np.min(region_indices[0])
            max_y = np.max(region_indices[0])
            
            # Compute width and height in grid cells.
            region_width_cells = max_x - min_x + 1
            region_height_cells = max_y - min_y + 1
            
            # Convert dimensions to meters.
            region_width_m = region_width_cells * resolution
            region_height_m = region_height_cells * resolution
            
            # Compute the total area of this cluster in square meters.
            # (Number of cells in the region) × (area of each cell).
            region_size_cells = len(region_indices[0])
            region_area_m2 = region_size_cells * (resolution ** 2)
            
            # Filter condition: mark region if it is too long in any dimension
            # or if its total area is below the minimum threshold.
            if (
                region_width_m > self.max_cone_length_m or
                region_height_m > self.max_cone_length_m or
                region_area_m2 < self.min_cone_area_m2
            ):
                data_array[region_indices] = -1
        
        filtered_map = OccupancyGrid()
        filtered_map.header = occupancy_map.header
        filtered_map.info = occupancy_map.info
        filtered_map.data = data_array.flatten().tolist()
        
        return filtered_map

    def transform_point_to_frame(self, point, header, target_frame):
        """
        Transform a Point from its original frame to a target frame using TF.
        
        :param point: The geometry_msgs/Point to transform.
        :param header: The header (containing frame_id and stamp) of the point.
        :param target_frame: The target frame to transform the point into.
        :return: The transformed point or None if the transform failed.
        """
        try:
            stamp_time = rclpy.time.Time.from_msg(header.stamp)
            transform_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                header.frame_id,
                stamp_time,
                timeout=Duration(seconds=0.5)
            )
            ps = PointStamped()
            ps.header = header
            ps.point = point
            pt_map = tf2_geometry_msgs.do_transform_point(ps, transform_stamped)
            return pt_map
        except Exception as e:
            self.get_logger().warn(f"Failed to transform cone to {target_frame}: {e}")
            return None

    def world_to_grid(self, x, y, origin_x, origin_y, resolution):
        """
        Convert world/odom coordinates (x_world, y_world) to grid indices (gx, gy)
        based on the origin and resolution of the occupancy grid.
        
        :param x: The world x-coordinate.
        :param y: The world y-coordinate.
        :param origin_x: The x-coordinate of the map origin.
        :param origin_y: The y-coordinate of the map origin.
        :param resolution: The map resolution.
        :return: A tuple (grid_x, grid_y).
        """
        gx = int((x - origin_x) / resolution)
        gy = int((y - origin_y) / resolution)
        return (gx, gy)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
