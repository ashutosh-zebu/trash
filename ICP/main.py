#!/usr/bin/env python3

import open3d as o3d
import cv2
import numpy as np
import glob
import time
import IterativeClosestPoint 

folder = "/home/ashutosh/Documents/ICP/yoyo"
fx = 525.0  # focal length x 
fy = 525.0  # focal length y 
cx = 319.5  # optical center x 
cy = 239.5  # optical center y
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]], dtype=np.float32)



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
    pointcloud = np.genfromtxt('PCL_data/bunny.csv', delimiter=',', skip_header=1)
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

    # source_points = PCL()
    # target_points = source_points.copy()
    # target_points[:, 0] += 1
    source_points = np.array([[ 0.4481007,  -0.20743543,  1.0371771 ],
        [ 2.3701224,   5.185886,   -0.6914515 ],
        [ 3.1732314,   0.59267265, -0.59267265],
        [ 1.0833918,   0.03457257, -0.3802983 ],
        [ 1.1853395,   0.3062142,  -0.34449098],
        [ 1.3035367,   0.5334054,  -0.4445045 ],
        [ 1.4049746,  -0.9853183,   0.15557657],
        [ 2.491438,  -1.1581811,  -0.08272722],
        [ 2.4720905,  -1.0322382,   1.5952774 ],
        [ 3.6776173,  -1.1606506,   0.        ],
        [ 3.6973228,  -1.1223739,   1.4964986 ],
        [ 4.815675,   -1.3557389,  -0.15063764],
        [ 4.84593,    -1.0976793,   1.5681131 ],
        [ 5.4127574,  -0.6716957,  -1.1754675 ],
        [ 5.890354,   -1.461926,   -0.18274075],
        [ 5.9858985,  -1.3310441,   1.5211931 ],
        [ 7.1240144,  -0.8643143,  -0.21607858],
        [ 7.1673903,  -0.0,         1.778018  ],
        [ 6.857931,   -1.4742733,  -4.177108  ],
        [ 2.2732527,   2.0101483,  -0.09137037]])
    
    target_points = np.array([[ 0.40759203, -0.06914514 , 1.0717498 ],
        [ 1.4189316,   0.31115317 ,-0.2667027 ],
        [ 1.4097518,  -0.9853183  , 0.10371771],
        [ 2.4018106,  -0.07284936 ,-0.29139742],
        [ 1.8124583,  -1.6026858  , 0.07284936],
        [ 2.491438 ,  -1.1581811  ,-0.08272722],
        [ 2.4720905,  -1.0322382  , 1.5952774 ],
        [ 3.6371012,  -1.2767156  ,-0.11606506],
        [ 3.6607156,  -1.1112611  , 1.4816816 ],
        [ 4.7762017,  -1.3446261  ,-0.1494029 ],
        [ 4.856068 ,  -1.2544905  , 1.4113019 ],
        [ 5.372958 ,  -0.6667568  ,-1.1668245 ],
        [ 5.932721 ,  -1.2791852  ,-0.18274075],
        [ 6.0310307,  -1.3310441  , 1.3310441 ],
        [ 7.1240144,  -0.8643143  ,-0.21607858],
        [ 7.1421804,  -1.3112882  , 0.0       ],
        [ 7.1673903,  -0.0        , 1.778018  ],
        [ 6.9061813,  -1.228561   ,-4.177108  ],
        [ 1.137401 ,   0.38276777 ,-0.42104453]])

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
    R_est, t_est = IterativeClosestPoint.ICP(
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