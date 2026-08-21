import open3d as o3d
import cv2
import numpy as np
import glob
from scipy.spatial import KDTree
import os
import time
import test2 

folder = "/home/ashutosh/Documents/ICP/yoyo"
fx = 525.0  # focal length x 
fy = 525.0  # focal length y 
cx = 319.5  # optical center x 
cy = 239.5  # optical center y
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float32)


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

def depth_to_point_cloud(depth, K, image_bgr=None):
    H, W = depth.shape
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    # Create meshgrid of pixel coordinates
    u = np.arange(W)
    v = np.arange(H)
    u_grid, v_grid = np.meshgrid(u, v, indexing='xy')  # (H, W)

    # Unproject depth to 3D points
    z = depth
    x = (u_grid - cx) * z / fx
    y = (v_grid - cy) * z / fy

    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)

    # Optional: get colors from image
    colors = None
    if image_bgr is not None:
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        colors = image_rgb.reshape(-1, 3).astype(np.float32) / 255.0

    return points, colors

def PCL():
    # pointcloud = np.genfromtxt('bunny.csv',delimiter=',')
    pointcloud = np.genfromtxt('bunny.csv', delimiter=',', skip_header=1)
    np.set_printoptions(threshold=np.inf)
    print(pointcloud)
    return pointcloud

# def main():
#     files = sorted(glob.glob(os.path.join(folder, "*.png")))

#     for i in range(len(files) - 1):
#         image_processing_start = time.time()
#         img1 = files[i]
#         img2 = files[i+1]

#         depth1 = cv2.imread(img1, cv2.IMREAD_UNCHANGED).astype(np.float32)
#         depth2 = cv2.imread(img2, cv2.IMREAD_UNCHANGED).astype(np.float32)

#         # Convert depth to point clouds
#         source_points, _ = depth_to_point_cloud(depth1, K)
#         target_points, _ = depth_to_point_cloud(depth2, K)

#         print(source_points.shape)
#         print(target_points.shape)
#         image_processing_end = time.time()
#         print(f"time taken to pre process the image : {image_processing_end-image_processing_start}")

#         start =time.time()
#         # Run ICP
#         R_est, t_est = ICP(
#             source=source_points,
#             target=target_points,
#             rotation_matrix=np.eye(3),
#             translation_vector=np.zeros(3),
#             iterations=25
#         )
#         aligned_source = (R_est @ source_points.T).T + t_est

#         compute = time.time()
#         print(f"total time taken by icp :{compute - start}")
#         # Apply transformation to source
        

#         # Build Open3D point clouds
#         pcd_source = o3d.geometry.PointCloud()
#         pcd_source.points = o3d.utility.Vector3dVector(source_points)
#         pcd_source.paint_uniform_color([0, 1, 1])  # red

#         # Build Open3D point clouds
#         pcd_transformed = o3d.geometry.PointCloud()
#         pcd_transformed.points = o3d.utility.Vector3dVector(aligned_source)
#         pcd_transformed.paint_uniform_color([1, 0, 0])  # red

#         pcd_target = o3d.geometry.PointCloud()
#         pcd_target.points = o3d.utility.Vector3dVector(target_points)
#         pcd_target.paint_uniform_color([0, 1, 0])  # green

#         o3d.visualization.draw_geometries([pcd_source, pcd_transformed, pcd_target])
#         # o3d.io.write_point_cloud("output1.pcd", pcd_source)
#         # o3d.io.write_point_cloud("output2.pcd", pcd_target)
#         # o3d.visualization.draw_geometries([pcd_source, pcd_target])


def main():

    image_processing_start = time.time()

    source_points = PCL()
    target_points = source_points.copy()
    target_points[:, 0] += 1

    # depth1 = cv2.imread("/home/ashutosh/Documents/ICP/yoyo/0.png", cv2.IMREAD_UNCHANGED).astype(np.float32)
    # depth2 = cv2.imread("/home/ashutosh/Documents/ICP/yoyo/1.png", cv2.IMREAD_UNCHANGED).astype(np.float32)

    # Convert depth to point clouds
    # source_points, _ = depth_to_point_cloud(depth1, K)
    # target_points, _ = depth_to_point_cloud(depth2, K)

    print(source_points.shape)
    print(target_points.shape)
    image_processing_end = time.time()
    print(f"time taken to pre process the image : {image_processing_end-image_processing_start}")

    start =time.time()
    # Run ICP
    R_est, t_est = ICP(
        source=source_points,
        target=target_points,
        rotation_matrix=np.eye(3),
        translation_vector=np.zeros(3),
        iterations=18
    )
    aligned_source = (R_est @ source_points.T).T + t_est

    compute = time.time()
    print(f"total time taken by icp :{compute - start}")
    print(f"source point cloud size : {source_points.shape}")
    print(f"target point cloud size : {target_points.shape}")
    # Apply transformation to source
    

    # Build Open3D point clouds
    pcd_source = o3d.geometry.PointCloud()
    pcd_source.points = o3d.utility.Vector3dVector(source_points)
    pcd_source.paint_uniform_color([0, 1, 1])  # red

    # Build Open3D point clouds
    pcd_transformed = o3d.geometry.PointCloud()
    pcd_transformed.points = o3d.utility.Vector3dVector(aligned_source)
    pcd_transformed.paint_uniform_color([1, 0, 0])  # red

    pcd_target = o3d.geometry.PointCloud()
    pcd_target.points = o3d.utility.Vector3dVector(target_points)
    pcd_target.paint_uniform_color([0, 1, 0])  # green

    o3d.visualization.draw_geometries([pcd_source, pcd_transformed, pcd_target])
    # o3d.io.write_point_cloud("output1.pcd", pcd_source)
    # o3d.io.write_point_cloud("output2.pcd", pcd_target)
    # o3d.visualization.draw_geometries([pcd_source, pcd_target])


if __name__=="__main__":
    main()