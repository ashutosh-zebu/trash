import math
import numpy as np
from scipy.spatial import KDTree
import time

# def ICP(source, target, iterations=20, rotation_matrix=np.eye(3), translation_vector=np.zeros(3)):

#     tree = KDTree(target)
#     for i in range(iterations):
#         # Step 1: Transform source
#         transformed_source = (rotation_matrix @ source.T).T + translation_vector

#         # Step 2: Find correspondences
#         distance, indices = tree.query(transformed_source)
#         closest_points = target[indices]

#         # Step 3: Estimating Transformation
#         centroid_src = np.mean(transformed_source, axis=0)
#         centroid_tgt = np.mean(closest_points, axis=0)

#         src_centered = transformed_source - centroid_src
#         tgt_centered = closest_points - centroid_tgt

#         # Compute covariance and SVD
#         H = src_centered.T @ tgt_centered
#         U, _, Vt = np.linalg.svd(H)
#         R = Vt.T @ U.T
#         if np.linalg.det(R) < 0:
#             Vt[-1, :] *= -1
#             R = Vt.T @ U.T
#         t = centroid_tgt - R @ centroid_src

#         # Step 6: Update total transform
#         rotation_matrix = R @ rotation_matrix
#         translation_vector = R @ translation_vector + t
#         # print(rotation_matrix)
    
#     return rotation_matrix, translation_vector 


def ICP(source, target, iterations=20, rotation_matrix=np.eye(3), translation_vector=np.zeros(3)):
    print("tree")
    tree = KDTree(target)
    print("tree created")
    for i in range(iterations):
        # Step 1: Transform source
        transformed_source = (rotation_matrix @ source.T).T + translation_vector
        print("yoyo1")
        search_points_start = time.time()
        # Step 2: Find correspondences
        distance, indices = tree.query(transformed_source)
        closest_points = target[indices]
        search_points_end = time.time()
        print(f"time taken to check the check in the tree : {search_points_end - search_points_start}")
        print("yoyo2")
        # Step 3: Estimating Transformation
        centroid_src = np.mean(transformed_source, axis=0)
        centroid_tgt = np.mean(closest_points, axis=0)
        print("yoyo3")
        src_centered = transformed_source - centroid_src
        tgt_centered = closest_points - centroid_tgt
        print("yoyo4")
        # Compute covariance and SVD
        H = src_centered.T @ tgt_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = centroid_tgt - R @ centroid_src

        # Step 6: Update total transform
        rotation_matrix = R @ rotation_matrix
        translation_vector = R @ translation_vector + t
        # print(rotation_matrix)
        print("yoyo5")
    return rotation_matrix, translation_vector 