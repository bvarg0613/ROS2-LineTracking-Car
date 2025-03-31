import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2 as cv
import numpy as np

class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker_node')

        # Camera setup
        self.camera_index = 0
        self.cap = cv.VideoCapture("/dev/video0", cv.CAP_V4L2)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 340)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 220)

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open camera.")
            exit(1)

        self.get_logger().info("Camera opened successfully.")

        # Create publisher for control decision
        self.publisher_ = self.create_publisher(String, 'line_tracker', 10)

        # Set row index to scan and pixel to analyze
        self.scan_row_number = 200
        self.pixel_index = 320

        # Set timer to run the main logic periodically
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz

    def create_mask(self, image):
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        color = 105
        lower = np.array([color - 15, 80, 80])
        upper = np.array([color + 15, 255, 255])
        mask = cv.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv.erode(mask, kernel, iterations=2)
        mask = cv.dilate(mask, kernel, iterations=2)
        return mask

    def scan_row(self, mask, row_number):
        if row_number < 0 or row_number >= mask.shape[0]:
            self.get_logger().error("Row number out of bounds!")
            return None
        row = mask[row_number, :]
        return (row > 0).astype(int)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame.")
            return

        mask = self.create_mask(frame)
        scanned_row = self.scan_row(mask, self.scan_row_number)

        if scanned_row is None:
            return

        pixel_value = scanned_row[self.pixel_index]
        msg = String()

        if pixel_value == 1:
            msg.data = "STAY STRAIGHT, NOT GAY"
        else:
            # Check where the tape moved by scanning left and right
            left_indices = np.where(scanned_row[:self.pixel_index] == 1)[0]
            right_indices = np.where(scanned_row[self.pixel_index+1:] == 1)[0]

            if len(left_indices) > 0 and (len(right_indices) == 0 or left_indices[-1] > right_indices[0]):
                msg.data = "MOVE LEFT IDIOTTT"
            elif len(right_indices) > 0:
                msg.data = "MOVE RIGHT IDIOTTT"
            else:
                msg.data = "I'M LOST BRO"

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

        # Visualization
        cv.line(frame, (0, self.scan_row_number), (frame.shape[1], self.scan_row_number), (0, 255, 0), 1)
        cv.circle(frame, (self.pixel_index, self.scan_row_number), 4, (255, 255, 255), -1)
        cv.imshow("Camera", frame)
        cv.imshow("Mask", mask)
        cv.waitKey(1)


    def destroy_node(self):
        self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
