#!/usr/bin/env python3
from racecar_msgs.msg._semantic_grid import SemanticGrid
import rclpy
from rclpy.node import Node

import numpy as np
import numpy.typing as npt
from scipy.ndimage import label
from scipy.spatial import Delaunay

from avai_lab import enums

class TrackPolygonNode(Node):
    """
    Node that transforms the semantic grid into a track polygon
    """
    def __init__(self):
        super().__init__("track_polygon_node")
        self.declare_parameter("semantic_grid_topic", "/semantic_map")

        self.semantic_grid_subscriber = self.create_subscription(SemanticGrid, self.get_parameter("semantic_grid_topic").value,
                                                                 self.semantic_grid_callback, 10)

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
    
    def alpha_shape(points, alpha, only_outer = True):
        """
        Compute concave hull of a set of points
        :param points: np.array of shape (n,2)
        :param alpha: alpha value
        :param only_outer: boolean to specify if only outer border is kept or also inner edges
        :return: set of (i,j) pairs representing the edges of the convex hull (i,j) are indices in the points array
        """
        assert points.shape[0] > 3, "Need at least four points"

        def add_edge(edges, i, j):
            """
            Add an edge between i-th and j-th points if not already in list
            """
            if (i, j) in edges or (j, i) in edges:
                # edge already added
                assert (j, i) in edges, "Can't go over same directed edge twice"
                if only_outer:
                    # if both neighboring triangles are in shape, it's not a boundary edge
                    edges.remove((j, i))
                return
            edges.add((i, j))

        triangles = Delaunay(points)
        edges = set()
        # Loop over triangles
        # ia, ib, ic are indices of corner points of triangle
        for ia, ib, ic in triangles.vertices:
            pa = points[ia]
            pb = points[ib]
            pc = points[ic]
            # Compute radius of triangle circumcircle
            a = np.sqrt((pa[0] - pb[0]) ** 2 + (pa[1] - pb[1]) ** 2)
            b = np.sqrt((pb[0] - pc[0]) ** 2 + (pb[1] - pc[1]) ** 2)
            c = np.sqrt((pc[0] - pa[0]) ** 2 + (pc[1] - pa[1]) ** 2)
            s = (a + b + c) / 2.0
            area = np.sqrt(s * (s - a) * (s - b) * (s - c))
            circum_r = a * b * c / (4.0 * area)
            if circum_r < alpha:
                # add triangle to edges
                add_edge(edges, ia, ib)
                add_edge(edges, ib, ic)
                add_edge(edges, ic, ia)
        return edges

    def semantic_grid_callback(self, msg: SemanticGrid):
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
        cone_positions = np.array(cone_positions)
        outer_edges = self.alpha_shape(cone_positions, alpha = 0.25, only_outer = True)
        # todo: remove outer points from point set
        # todo: retrieve concave hull from inner points
        # todo: donut-shaped polygon (outer \ inner)

def main(args=None):
    rclpy.init(args=args)

    node = TrackPolygonNode()
    rclpy.spin(node) # used to loop the node

    rclpy.shutdown()

if __name__ == "__main__":
    main()