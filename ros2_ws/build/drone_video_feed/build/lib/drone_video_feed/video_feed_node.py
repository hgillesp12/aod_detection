import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import time
import warnings

# Suppress warnings
warnings.filterwarnings("ignore", category=FutureWarning)

# Load YOLOv5 model with custom weights
model = torch.hub.load('/Users/hannahgillespie/aod_detection/model/yolov5', 
                       'custom', 
                       path='/Users/hannahgillespie/aod_detection/model/best.pt', 
                       source='local')

# Define custom colors for each class
custom_colors = {
    'non_oak': (128, 128, 128),        # Grey
    'low_risk_oak': (0, 255, 0),       # Green
    'medium_risk_oak': (0, 255, 255),  # Yellow
    'high_risk_oak': (0, 0, 255)       # Red
}

class VideoFeedNode(Node):
    def __init__(self):
        super().__init__('video_feed_node')

        # Subscribe to the input image topic
        self.subscription = self.create_subscription(
            Image,
            '/quadcopter_with_camera/camera_sensor/image_raw',  # Input image topic
            self.image_callback,
            10
        )

        # Publisher for the output image with bounding boxes
        self.publisher_ = self.create_publisher(
            Image,
            '/processed_image_with_bboxes',  # Output image topic
            10
        )

        self.bridge = CvBridge()
        self.get_logger().info('Video Feed Node has been started')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image with your machine learning model
        processed_image = self.process_image(cv_image)

        # Convert the processed image back to ROS Image message
        output_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')

        # Publish the processed image with bounding boxes
        self.publisher_.publish(output_msg)

    def process_image(self, image):
        """
        Process the input image using YOLOv5 model and draw bounding boxes for detected objects.
        """
        # Start time for FPS calculation
        start_time = time.time()

        # Perform inference using the YOLOv5 model
        results = model(image)

        # Extract bounding boxes, class names, and confidences
        for det in results.xyxy[0]:  # Loop through detections
            xyxy = det[:4]  # Bounding box coordinates
            conf = det[4]   # Confidence score
            cls_id = int(det[5])  # Class ID

            # Get the class name
            class_name = results.names[cls_id]
            # Get the color for the class name
            color = custom_colors.get(class_name, (255, 255, 255))  # Default to white

            # Draw the bounding box
            cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)

            # Draw the label
            label = f'{class_name} {conf:.2f}'
            cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Calculate and display FPS
        fps = 1 / (time.time() - start_time)
        cv2.putText(image, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return image

def main(args=None):
    rclpy.init(args=args)
    node = VideoFeedNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
