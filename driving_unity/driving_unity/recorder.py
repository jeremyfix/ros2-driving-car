# coding: utf-8

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from driving_unity_interface.msg import CarCommand
from cv_bridge import CvBridge
import numpy as np


class Recorder(Node):
    def __init__(self):
        super().__init__("recorder")

        self.last_frame = None
        self.last_command = None
        self.chunk_size = 100

        self.all_commands = []
        self.all_frames = []
        self.chunk_idx = 0

        self.bridge = CvBridge()

        self.sub_commad = self.create_subscription(
            CarCommand, "/command", self.on_command, qos_profile=1
        )
        self.sub_image = self.create_subscription(
            CompressedImage, "/image_raw/compressed", self.on_image, qos_profile=1
        )

    def on_image(self, msg):
        self.last_frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.save()

    def on_command(self, msg):
        self.last_command = (msg.throttle, msg.steering)
        self.save()

    def save(self):
        if self.last_frame is None or self.last_command is None:
            return

        self.all_commands.append(self.last_command)
        self.all_frames.append(self.last_frame)

        if len(self.all_commands) == self.chunk_size:
            # Dump them on disk
            chunk_filepath = f"chunk-{self.chunk_idx:05d}.npz"
            self.get_logger().info("Dumping chunk ")
            np.savez_compressed(
                chunk_filepath, frames=self.all_frames, commands=self.all_commands
            )
            self.chunk_idx += 1

            self.all_commands = []
            self.all_frames = []


def main(args=None):
    rclpy.init(args=args)

    recorder = Recorder()
    rclpy.spin(recorder)

    recorder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # This is the main thread, thread #1
    main()
