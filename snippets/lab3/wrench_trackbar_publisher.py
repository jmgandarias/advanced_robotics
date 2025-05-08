import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import tkinter as tk
from threading import Thread
import time

class WrenchTrackbarPublisher(Node):
    def __init__(self):
        super().__init__('wrench_trackbar_publisher')
        self.publisher_ = self.create_publisher(Wrench, 'external_wrenches', 10)
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.torque_x = 0.0
        self.torque_y = 0.0
        self.torque_z = 0.0
        self.continuous_mode = True

        self.root = tk.Tk()
        self.root.title("Wrench Publisher")

        self.create_trackbar('Force X', -30.0, 30.0, self.update_force_x)
        self.create_trackbar('Force Y', -30.0, 30.0, self.update_force_y)
        self.create_trackbar('Force Z', -30.0, 30.0, self.update_force_z)
        self.create_trackbar('Torque X', -10.0, 10.0, self.update_torque_x)
        self.create_trackbar('Torque Y', -10.0, 10.0, self.update_torque_y)
        self.create_trackbar('Torque Z', -10.0, 10.0, self.update_torque_z)

        self.mode_button = tk.Button(self.root, text="Switch to Instantaneous Mode", command=self.switch_mode)
        self.mode_button.pack()

        center_button = tk.Button(self.root, text="Center All", command=self.center_all)
        center_button.pack()

        # Start the publishing thread
        self.publish_thread = Thread(target=self.publish_wrench_continuously)
        self.publish_thread.start()

    def create_trackbar(self, label, min_val, max_val, callback):
        frame = tk.Frame(self.root)
        frame.pack()
        tk.Label(frame, text=f"{min_val}").pack(side=tk.LEFT)
        trackbar = tk.Scale(frame, label=label, from_=min_val, to=max_val, orient=tk.HORIZONTAL, command=callback, resolution=0.1, length=400, sliderlength=30)
        trackbar.pack(side=tk.LEFT)
        trackbar.bind("<ButtonRelease-1>", lambda event, lbl=label: self.reset_slider(lbl))
        tk.Label(frame, text=f"{max_val}").pack(side=tk.LEFT)

    def update_force_x(self, value):
        self.force_x = float(value)

    def update_force_y(self, value):
        self.force_y = float(value)

    def update_force_z(self, value):
        self.force_z = float(value)

    def update_torque_x(self, value):
        self.torque_x = float(value)

    def update_torque_y(self, value):
        self.torque_y = float(value)

    def update_torque_z(self, value):
        self.torque_z = float(value)

    def reset_slider(self, label):
        if not self.continuous_mode:
            for widget in self.root.winfo_children():
                if isinstance(widget, tk.Frame):
                    for child in widget.winfo_children():
                        if isinstance(child, tk.Scale) and child.cget("label") == label:
                            child.set(0.0)

    def switch_mode(self):
        self.continuous_mode = not self.continuous_mode
        mode_text = "Switch to Continuous Mode" if not self.continuous_mode else "Switch to Instantaneous Mode"
        self.mode_button.config(text=mode_text)

    def center_all(self):
        for widget in self.root.winfo_children():
            if isinstance(widget, tk.Frame):
                for child in widget.winfo_children():
                    if isinstance(child, tk.Scale):
                        child.set(0.0)

    def publish_wrench_continuously(self):
        while rclpy.ok():
            msg = Wrench()
            msg.force.x = self.force_x
            msg.force.y = self.force_y
            msg.force.z = self.force_z
            msg.torque.x = self.torque_x
            msg.torque.y = self.torque_y
            msg.torque.z = self.torque_z
            self.publisher_.publish(msg)
            time.sleep(0.001)  # Sleep for 1 millisecond (1 kHz frequency)

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = WrenchTrackbarPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()