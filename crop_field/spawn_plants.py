#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import subprocess
import time

from .patch_node import generate_explicit_non_overlapping_patches

class PlantSpawner(Node):
    def __init__(self):
        super().__init__('plant_spawner')

        FIELD_SIZE = 20
        PATCH_SIZE = 2
        NUM = 50

        field, _ = generate_explicit_non_overlapping_patches(FIELD_SIZE, PATCH_SIZE, NUM)

        positions = []
        for r in range(FIELD_SIZE):
            for c in range(FIELD_SIZE):
                if field[r][c] == 1:
                    positions.append((r, c))

        self.get_logger().info(f"Spawning {len(positions)} grass patches in Gazebo")

        for (x, y) in positions:
            spawn_cmd = [
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", f"grass_{x}_{y}",
                "-file", "/home/aditipk/.gazebo/models/corn_plant/model.sdf", 
                "-x", str(x),
                "-y", str(y),
                "-z", "0"
            ]
            subprocess.Popen(spawn_cmd)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PlantSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

