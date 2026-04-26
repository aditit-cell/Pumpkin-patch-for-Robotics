#!/usr/bin/env python3
import numpy as np
import random

def generate_explicit_non_overlapping_patches(field_size, patch_size, num_patches_target):
    N = field_size
    S = patch_size
    rng = random.SystemRandom()

    field = np.zeros((N, N))
    occupied_points = set()
    patches_placed = 0
    MAX_ATTEMPTS = 50000
    attempts = 0
    valid_start_indices = np.arange(N - S + 1)

    while patches_placed < num_patches_target and attempts < MAX_ATTEMPTS:
        attempts += 1
        r_start = rng.choice(valid_start_indices)
        c_start = rng.choice(valid_start_indices)
        potential_new_points = set()
        is_overlapping = False

        for r in range(r_start, r_start + S):
            for c in range(c_start, c_start + S):
                point = (r, c)
                if point in occupied_points:
                    is_overlapping = True
                    break
                potential_new_points.add(point)
            if is_overlapping:
                break

        if not is_overlapping:
            field[r_start:r_start+S, c_start:c_start+S] = 1
            occupied_points.update(potential_new_points)
            patches_placed += 1

    return field, patches_placed

