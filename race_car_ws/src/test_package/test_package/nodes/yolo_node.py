import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloConeDetector(Node):
    """This node uses a trained yolo model to detect cones in images received by subscribing topics."""
    def __init__(self):
        super().__init__('yolo_cone_detector')
        self.subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/myuser/Desktop/race-car-buggy-busters/race_car_ws/src/test_package/runs/detect/train/weights/best.pt')
        self.colors = {
            "yellow_cones": (0, 255, 255),  # Yellow
            "orange_cones": (0, 165, 255),  # Orange
            "blue_cones": (255, 0, 0)       # Blue
        }

        self.current_depth_image = None
        self.depth_image_sub = self.create_subscription(
            Image,
            '/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )


    def camera_callback(self, msg):
        """Outputs message information of a certain topic."""
        msg_str = f"""
        Header:
            Frame ID: {msg.header.frame_id}
            Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns
        Resolution:
            Width: {msg.width}
            Height: {msg.height}
        Distortion Model: {msg.distortion_model}
        Distortion Coefficients: {msg.d}
        Intrinsic Matrix (K): {msg.k.tolist()}
        Rectification Matrix (R): {msg.r.tolist()}
        Projection Matrix (P): {msg.p.tolist()}
        """
        self.get_logger().info(msg_str)

    def cameraImage_callback(self, msg):
        bridge = CvBridge()

        # Convert ROS Image message to OpenCV image
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        msg_str = f"""
         Header:
         Frame ID: {msg.header.frame_id}
        Timestamp: {msg.header.stamp.sec}s {msg.header.stamp.nanosec}ns
        Resolution:
         Width: {cv_image.shape[1]} pixels
         Height: {cv_image.shape[0]} pixels
        Channels: {cv_image.shape[2] if len(cv_image.shape) > 2 else 1}
        """
        self.get_logger().info(msg_str)
    
    def image_callback(self, msg):
        """Converts images to OpenCV and passes them to the yolo model."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # Run YOLOv8 detection
        results = self.model(cv_image)
        
        # Diplay results
        self.visualize_results(cv_image, results[0])

    def depth_image_callback(self, msg):
        """Converts images to apporpriate format to extract depth of specific points."""
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

    def visualize_results(self, image, results):
        """Visualizes the images after detecting and calculating depth information."""
        if self.current_depth_image is None:
            self.get_logger().warning("No depth image received yet.")
            return
        # Draw bounding boxes on the cones
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2  # Center of the bounding box
            
            try:
                depth_value = self.current_depth_image[cy, cx]
            except IndexError:
                self.get_logger().warning(f"Depth value out of bounds for box at ({cx}, {cy})")
                continue

            self.get_logger().info(f"Detected {results.names[int(box.cls[0])]} at distance: {depth_value:.2f} milimeters")
            label = f"{results.names[int(box.cls[0])]} {depth_value:.2f}mm"
            color = self.colors.get(label, (0, 255, 0))
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Display image
        cv2.imshow('YOLO Detection', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloConeDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
