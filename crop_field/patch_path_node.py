import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import random
import math

# ---------------- PATCH GENERATION LOGIC ---------------- #

def make_patches(size_of_field, patch_size, count, start_xy=None):
    grid_size = size_of_field
    patch_dim = patch_size
    
    grid = np.zeros((grid_size, grid_size), dtype=int)
    claimed = set()
    patch_data = []
    patches_done = 0
    max_tries = 50000
    
    valid_starts = np.arange(grid_size - patch_dim + 1)

    def record_patch(r_idx, c_idx, ID):
        nonlocal patches_done
        
        center_off = (patch_dim - 1) / 2.0
        x_center = r_idx + center_off 
        y_center = grid_size - (c_idx + center_off)
        
        points = set()
        for r in range(r_idx, r_idx + patch_dim):
            for c in range(c_idx, c_idx + patch_dim):
                points.add((r, c))
        claimed.update(points)

        patch_data.append({
            'id': ID, 
            'r_idx': r_idx, 
            'c_idx': c_idx,
            'y': y_center, 
            'x': x_center  
        })
        patches_done += 1

    if start_xy is not None and count >= 1:
        r_idx, c_idx = start_xy
        if r_idx in valid_starts and c_idx in valid_starts:
            record_patch(r_idx, c_idx, 1)

    tries = 0
    while patches_done < count and tries < max_tries:
        tries += 1
        r_idx = np.random.choice(valid_starts)
        c_idx = np.random.choice(valid_starts)

        is_clash = False
        current_points = set()

        for r in range(r_idx, r_idx + patch_dim):
            for c in range(c_idx, c_idx + patch_dim):
                if (r, c) in claimed:
                    is_clash = True
                    break
                current_points.add((r, c))
            if is_clash:
                break

        if not is_clash:
            record_patch(r_idx, c_idx, patches_done + 1)

    for p in patch_data:
        r, c, ID = p['r_idx'], p['c_idx'], p['id']
        grid[r:r + patch_dim, c:c + patch_dim] = ID

    return grid, patch_data

def find_path(patches):
    available = patches[:] 
    sorted_path = []

    while available:
        available.sort(key=lambda p: (-p['y'], p['x']))
        
        current = available.pop(0)
        sorted_path.append(current)
        path_active = True
        
        while path_active and available:
            candidates = []
            current_x = current['x']
            current_y = current['y']
            for next_p in available:
                next_x = next_p['x']
                if next_x > current_x:
                    next_y = next_p['y']
                    y_diff = abs(next_y - current_y) 
                    dist = math.dist((current_y, current_x), (next_y, next_x))
                    candidates.append((y_diff, dist, next_p))
            
            if not candidates:
                path_active = False
                continue

            candidates.sort(key=lambda x: (x[0], x[1]))
            
            next_p = candidates[0][2]
            sorted_path.append(next_p)
            available.remove(next_p)
            current = next_p 

    renumbered = []
    for i, p in enumerate(sorted_path):
        new_p = p.copy()
        new_p['id'] = i + 1 
        renumbered.append(new_p)
            
    return renumbered

# ---------------- ROS2 NODE ---------------- #

class PatchPathNode(Node):
    def __init__(self):
        super().__init__("patch_path_node")
        self.publisher = self.create_publisher(String, '/crop_patch_path', 10)
        self.get_logger().info("Patch Path Node Started!")

        grid, patches = make_patches(30, 5, 5)
        path = find_path(patches)

        msg = String()
        msg.data = str(path)
        self.publisher.publish(msg)
        self.get_logger().info("Published patch path!")

def main(args=None):
    rclpy.init(args=args)
    node = PatchPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

