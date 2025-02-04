import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detection')

        # Subscribe to the USB camera topic
        self.subscription = self.create_subscription(
            Image, '/camera1/image_raw', self.image_callback, 3)
        self.subscription  # Prevents unused variable warning

        # Publisher for the processed image
        self.publisher = self.create_publisher(Image, '/out/image', 3)

        # Bridge for converting ROS <-> OpenCV images
        self.bridge = CvBridge()

        # Load the Haar Cascade model for face detection
        model_path = '/home/parallels/ros2_ws/src/image_processor/image_processor/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(model_path)

    def image_callback(self, data):
        """ Convert ROS image to OpenCV format, detect faces, and publish output """

        try:
            # Convert ROS Image to OpenCV image
            imCV = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

            # Convert to grayscale for face detection
            gray = cv2.cvtColor(imCV, cv2.COLOR_BGR2GRAY)

            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # Draw bounding boxes
            for (x, y, w, h) in faces:
                cv2.rectangle(imCV, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Convert the OpenCV image back to ROS format
            out_image = self.bridge.cv2_to_imgmsg(imCV, encoding="bgr8")

            # Publish the processed image
            self.publisher.publish(out_image)
            self.get_logger().info("Face(s) detected")

            # Show the processed image in a window
            # cv2.imshow("Face Detection", imCV)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
