import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import scipy.spatial
import copy
import time
import os



d = 0.45  # Min distance threshold for filtering depth data [meters]
dd = 0.15  # Max - Min distance threshold for filtering depth data [meters]
min_cluster_size_ratio = 0.081  # Ratio for minimum cluster size to the largest cluster size
normal_orientation_threshold = 0.3  # Threshold for normal orientation filtering



# import mesh from file from RGB
# meshRGB = o3d.geometry.TriangleMesh()
# meshRGB = o3d.io.read_triangle_mesh("/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/small_size.glb")
# meshRGB.scale(0.3, center=meshRGB.get_center()) # Scale the mesh down to 30% of its original size

# meshRGB.compute_vertex_normals()

#use Intel realSense D435 to take rgbd images with region of interest

pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

pipe.start(config)

time.sleep(8)  # Allow some time for the camera to warm up

threshold_filter = rs.threshold_filter()
threshold_filter.set_option(rs.option.min_distance, d)  # Set minimum distance
threshold_filter.set_option(rs.option.max_distance, d + dd)  # Set maximum distance



frames = pipe.wait_for_frames()
frames_filtered = threshold_filter.process(frames)
depth_image = np.asanyarray(frames_filtered.get_data())
pc = rs.pointcloud() 
points = pc.calculate(frames_filtered)


# Convert point cloud to Open3D format
pcd = o3d.geometry.PointCloud()
vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)  # xyz
pcd.points = o3d.utility.Vector3dVector(vtx)

#remove points that are more than 0.3 from x = 0, y = 0 at any z coordinate
pcd = pcd.select_by_index(np.where(np.linalg.norm(vtx[:, :2], axis=1) < 0.3)[0])
pipe.stop()

# use scipy.spatial to create mesh from pcd point cloud
# convert point cloud to numpy array
vtx = np.asarray(pcd.points)

# create a Delaunay triangulation from the point cloud
tri = scipy.spatial.Delaunay(vtx[:, :2])  # Use only x and y for triangulation
# create a mesh from the triangulation
mesh = o3d.geometry.TriangleMesh()
mesh.vertices = o3d.utility.Vector3dVector(vtx)
mesh.triangles = o3d.utility.Vector3iVector(tri.simplices)
mesh.compute_vertex_normals()
# reverse the triangle normals
#mesh.triangle_normals = o3d.utility.Vector3dVector(-np.asarray(mesh.triangle_normals))
mesh.triangles = o3d.utility.Vector3iVector(np.flip(np.asarray(mesh.triangles), axis=1))
mesh.compute_vertex_normals()
o3d.io.write_triangle_mesh("/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_depth.ply", mesh)



