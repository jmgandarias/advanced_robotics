#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tkinter as tk
import threading

class EquilibriumPosePublisher(Node):
    def __init__(self):
        super().__init__('equilibrium_pose_publisher')
        self.publisher_pose = self.create_publisher(PoseStamped, 'equilibrium_pose', 10)

        # Initial pose values
        self.current_pose_x = 1.3071
        self.current_pose_y = 0.701
        self.pending_pose_x = self.current_pose_x
        self.pending_pose_y = self.current_pose_y

        # Create GUI
        self.root = tk.Tk()
        self.root.title("Equilibrium Pose Publisher")

        self.slider_x = self.create_trackbar('Pose X', -1.6, 1.6, self.update_pending_pose_x)
        self.slider_y = self.create_trackbar('Pose Y', -1.6, 1.6, self.update_pending_pose_y)

        self.slider_x.set(self.pending_pose_x)
        self.slider_y.set(self.pending_pose_y)

        publish_button = tk.Button(self.root, text="Publish Equilibrium Pose", command=self.update_current_pose)
        publish_button.pack(pady=10)

        # Start publishing timer (300 Hz)
        self.create_timer(1.0 / 300.0, self.publish_equilibrium_pose)

    def create_trackbar(self, label, min_val, max_val, callback):
        frame = tk.Frame(self.root)
        frame.pack()
        tk.Label(frame, text=f"{min_val}").pack(side=tk.LEFT)
        trackbar = tk.Scale(frame, label=label, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                            command=callback, resolution=0.01, length=400, sliderlength=30)
        trackbar.pack(side=tk.LEFT)
        tk.Label(frame, text=f"{max_val}").pack(side=tk.LEFT)
        return trackbar

    def update_pending_pose_x(self, value):
        self.pending_pose_x = float(value)

    def update_pending_pose_y(self, value):
        self.pending_pose_y = float(value)

    def update_current_pose(self):
        self.current_pose_x = self.pending_pose_x
        self.current_pose_y = self.pending_pose_y
        self.get_logger().info(f'Updated pose to: X = {self.current_pose_x}, Y = {self.current_pose_y}')

    def publish_equilibrium_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = self.current_pose_x
        msg.pose.position.y = self.current_pose_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.publisher_pose.publish(msg)

    def run(self):
        # Start ROS spinning in a background thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()

        # Start the GUI main loop
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = EquilibriumPosePublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
