import octomap

# Load the OctoMap file (.bt or .ot)
tree = octomap.OcTree(b"/home/ashutosh/zebu_ws/src/drone_nav/drone_nav_path_planner/pcd/cloud_map.bt")

# Get resolution
print("Resolution:", tree.getResolution())

# Get bounding box
bbmin, bbmax = tree.getMetricMin(), tree.getMetricMax()
print("Bounding Box Min:", bbmin)
print("Bounding Box Max:", bbmax)
print(f"Size: {tree.getMetricSize()}")

# https://octomap.github.io/octomap/doc/classoctomap_1_1OcTreeNode.html

# Iterate over all occupied voxels
# occupied_node =[]
# free_node = []

# print("Occupied voxels:")
# for node in tree.begin_leafs():
#     # print(node)
#     if tree.isNodeOccupied(node):
#         coord = tree.keyToCoord(node.getKey()) 
#         # print(f"Voxel at {coord} with occupancy {node.getValue()}")
#         occupied_node.append(coord)
#     else:
#         coord = tree.keyToCoord(node.getKey()) 
#         free_node.append(coord)

# # print((24.75, 1442.85, 9.75).getKey())
# val = tree.search((24.75, 1442.85, 9.75),0)
# # print(val.getValue())

# if val is None:
#     print("error")
# else:
#     print("pass")





# import time 
# start = time.time()
# val = tree.search([-90.7, -6.7, -1.1], depth=0)
# print(f"Occupancy log-odds: {val}")
# print(f"Occupancy probability: {val.getOccupancy()}")
# print(f"Occupied?: {tree.isNodeOccupied(val)}")
# print(f"Has children?: {val.hasChildren()}")
# print(tree.isNodeOccupied(val))
# end = time.time()

# print(end-start)



# start = time.time()
# val = tree.search((-0.25, -0.25,  6.25), depth=0)
# print(val)
# print(val.getValue())
# print(tree.isNodeOccupied(val))
# end = time.time()

# print(end-start)








# # Iterate over all unoccupied voxels
# for node in tree.begin_leafs():
#     if not tree.isNodeOccupied(node):
#         coord = tree.keyToCoord(node.getKey()) 
#         # print(f"Voxel at {coord} with occupancy {node.getValue()}")
#         # print(coord)

with open("free_nodes.txt", "w") as file:
    for node in tree.begin_leafs():
        if not tree.isNodeOccupied(node):
            coord = tree.keyToCoord(node.getKey())
            rounded = list(round(c, 2) for c in coord)
            file.write(f"{rounded}\n")    
# points =[]
# print(tree.begin_leafs())
# # for node in tree.begin_leafs():
# #     if not tree.isNodeOccupied(node):
# #         coord = tree.keyToCoord(node.getKey()) 
# #         points.append(coord)
# print(dir(tree))
# print(tree.search((-89.65, -6.55, -1.05)).getValue())


# sorted_points = sorted(points, key=lambda p: (p[0], p[1], p[2]))

# for p in sorted_points:
#     print(p)

