#!/usr/bin/env python3
# spray_node.py
# Put this in your package (e.g. ~/ros2_ws/src/pumpkin_sprayer/pumpkin_sprayer/spray_node.py)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import random
import threading
import tkinter as tk
from tkinter import ttk

class SprayNode(Node):

    def __init__(self):
        super().__init__('spray_node')
        self.bridge = CvBridge()

        # Subscribe
        self.subscription = self.create_subscription(
            Image,
            '/downward_cam/image_raw',
            self.image_callback,
            10
        )

        # Manual override (None => AUTO)
        self.override_color = None

        # Spray counters and required mapping
        self.spray_counts = {"ORANGE":0,"GREEN":0,"YELLOW":0,"VIOLET":0,"BLACK":0}
        self.required_sprays = {"ORANGE":0,"GREEN":1,"YELLOW":3,"VIOLET":4,"BLACK":5}

        # Start GUI in background thread
        gui_thread = threading.Thread(target=self.start_gui, daemon=True)
        gui_thread.start()

        self.get_logger().info("Pumpkin sprayer node running (GUI mode).")

    # ---------------- GUI ----------------
    def start_gui(self):
        """
        Simple Tk GUI with buttons:
        AUTO, ORANGE, GREEN, YELLOW, VIOLET, BLACK
        """
        root = tk.Tk()
        root.title("Pumpkin Spray Control")
        root.geometry("260x320")

        label = ttk.Label(root, text="Select color (AUTO = HSV detect)", font=("Arial", 11))
        label.pack(pady=(10,6))

        btn_frame = ttk.Frame(root)
        btn_frame.pack(pady=4, padx=10, fill="both", expand=True)

        def add_button(text, color):
            b = ttk.Button(btn_frame, text=text,
                           command=lambda: self.set_override(color),
                           width=22)
            b.pack(pady=6)

        add_button("AUTO (HSV)", None)
        add_button("ORANGE", "ORANGE")
        add_button("GREEN", "GREEN")
        add_button("YELLOW", "YELLOW")
        add_button("VIOLET", "VIOLET")
        add_button("BLACK", "BLACK")

        # status label
        self.status_var = tk.StringVar()
        self.status_var.set("Mode: AUTO")
        status = ttk.Label(root, textvariable=self.status_var, font=("Arial", 10))
        status.pack(pady=(6,10))

        # update status periodically (from ROS thread)
        def periodic_status():
            mode = "AUTO" if self.override_color is None else self.override_color
            s = f"Mode: {mode}"
            # show spray counts summary
            s2 = " | ".join([f"{k}:{v}/{self.required_sprays[k]}" for k,v in self.spray_counts.items()])
            self.status_var.set(s + "\n" + s2)
            root.after(500, periodic_status)

        root.after(500, periodic_status)
        root.mainloop()

    def set_override(self, color):
        self.override_color = color
        if color is None:
            self.get_logger().info("GUI → AUTO HSV detection enabled")
        else:
            self.get_logger().info(f"GUI → Manual override: {color}")

    # ---------------- HSV classifier (AUTO) ----------------
    def classify_pumpkin_auto(self, frame):
        """
        Returns (color, unhealthy_level).
        Uses averaged HSV with tolerant ranges tuned for your pumpkins.
        """
        if frame is None or frame.size == 0:
            return "UNKNOWN", 0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mean across pixels -> (H,S,V)
        h, s, v = hsv.mean(axis=0).mean(axis=0)

        # Black: very low brightness
        if v < 35:
            return "BLACK", self.required_sprays["BLACK"]

        # Orange (approx H 8..18), require decent saturation
        if 6 <= h <= 20 and s > 80:
            return "ORANGE", self.required_sprays["ORANGE"]

        # Yellow (approx H 20..40)
        if 18 <= h <= 40 and s > 70 and v > 50:
            return "YELLOW", self.required_sprays["YELLOW"]

        # Green (approx H 40..90)
        if 40 <= h <= 95 and s > 70:
            return "GREEN", self.required_sprays["GREEN"]

        # Violet / purple (approx H 125..170)
        if (125 <= h <= 170) and s > 50:
            return "VIOLET", self.required_sprays["VIOLET"]

        return "UNKNOWN", 0

    # ---------------- Spray visualization ----------------
    def draw_spray_effect(self, frame, color_hint=None):
        """
        Draw a cone-of-spray in white. color_hint unused but kept for future.
        """
        h, w, _ = frame.shape
        origin = (w//2, int(h*0.06))

        # draw many small translucent dots to represent mist
        for _ in range(180):
            dx = int(random.uniform(-70, 70))
            dy = int(random.uniform(40, 260))
            x = origin[0] + dx
            y = origin[1] + dy
            if 0 <= x < w and 0 <= y < h:
                # small white dot
                cv2.circle(frame, (x, y), 2, (255,255,255), -1)

    # ---------------- Main image callback ----------------
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w, _ = frame.shape
        # crop center area (to reduce background influence)
        crop = frame[h//4:3*h//4, w//4:3*w//4]

        # Determine color (AUTO or manual override)
        if self.override_color is None:
            color, required = self.classify_pumpkin_auto(crop)
        else:
            color = self.override_color
            # safety: if override is an invalid key, treat as UNKNOWN
            required = self.required_sprays.get(color, 0)

        # ensure counters exist for color (avoid KeyError)
        if color not in self.spray_counts:
            self.spray_counts[color] = 0
            if color not in self.required_sprays:
                required = 0

        # Spray only if needed
        if required > 0 and self.spray_counts.get(color,0) < required:
            self.draw_spray_effect(frame, color_hint=color)
            self.spray_counts[color] = self.spray_counts.get(color,0) + 1

        # HUD overlays
        cv2.putText(frame, f"Color: {color}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

        cv2.putText(frame, f"Sprays: {self.spray_counts.get(color,0)}/{required}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        # also put small summary in lower-left
        summary = " ".join([f"{k}:{v}/{self.required_sprays[k]}" for k,v in self.spray_counts.items()])
        cv2.putText(frame, summary, (20, h-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)

        # show
        cv2.imshow("Pumpkin Sprayer View", frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = SprayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
