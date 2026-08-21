import math
import numpy as np
from scipy.spatial import KDTree

def ICP(source, target, depth = 0):
    s = []
    t = []
    closest_points = []
    curr_closest = math.inf
    for source_point in source:
        for target_point in target:
            depth = int(math.dist(source_point, target_point))
            if depth < curr_closest:
                curr_closest = depth
                s = source_point
                t = target_point
        closest_points.append([s, t, curr_closest])
        s = []
        t = []
        curr_closest = math.inf 
    return closest_points

# def ICP(source, 
#         target,
#         rotation_matrix=np.eye(3),
#         translation_vector=np.zeros(3),
#         iterations=1):
    
    
#     # Copy so we don't overwrite input
#     src = source.copy()
    
#     for _ in range(iterations):
#         # Step 1: Transform source with current estimate
#         transformed_source = (rotation_matrix @ src.T).T + translation_vector
        
#         # Step 2: Find nearest neighbors
#         tree = KDTree(target)
#         distances, indices = tree.query(transformed_source)
#         closest_points = target[indices]
        
#         # Step 3: Estimate new transformation using SVD
#         mu_src = np.mean(transformed_source, axis=0)
#         mu_dst = np.mean(closest_points, axis=0)

#         H = (transformed_source - mu_src).T @ (closest_points - mu_dst)
#         U, _, Vt = np.linalg.svd(H)
#         R = Vt.T @ U.T
#         if np.linalg.det(R) < 0:  # fix reflection
#             Vt[-1, :] *= -1
#             R = Vt.T @ U.T

#         t = mu_dst - R @ mu_src

#         # Update overall transformation
#         rotation_matrix = R @ rotation_matrix
#         translation_vector = R @ translation_vector + t

#         # Update source for next iteration
#         src = transformed_source
    
#     return rotation_matrix, translation_vector