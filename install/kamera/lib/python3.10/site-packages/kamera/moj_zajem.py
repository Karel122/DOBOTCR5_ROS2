import open3d as o3d
import numpy as np
import copy
import os


d = 0  # Min distance threshold for filtering depth data [meters]
dd = 1000  # Max - Min distance threshold for filtering depth data [meters]
min_cluster_size_ratio = 0.1  # Ratio for minimum cluster size to the largest cluster size
normal_orientation_threshold = 0.3  # Threshold for normal orientation filtering

def get_all_scanning_points(mesh_work, mesh_test_org):
    """
    Get scanning points from the mesh by finding clusters and placing cones at their center points.
    Args:
        mesh_work (o3d.geometry.TriangleMesh): The input mesh to work with.
        mesh_test_org (o3d.geometry.TriangleMesh): A test mesh to check for self-intersection.
    Returns:
        list: A list of scanning points, each containing the top point of cone, 
                normal vector pointing to the mesh and transformation matrix.
    """
    triangle_clusters, cluster_n_triangles, cluster_area = (
        mesh_work.cluster_connected_triangles())

    failed = []
    scanning_points = []
    #remove f"realsense_cones\\" directory if it exists

    max_index = -1
    for file in os.listdir("/home/lab-3d/realsense_cones"):
        index = int(file.split("_")[-1].split(".")[0])
        if index > max_index:
            max_index = index
    
    

    for cluster_index in range(len(cluster_n_triangles)):
        
        mesh_work.compute_triangle_normals()  # Ensure normals are computed before filtering
        center_point, normal_vector = find_cluster_centerpoint(mesh_work, triangle_clusters, cluster_index=cluster_index)

        z_axis = np.array([0, 0, 1])
        rotation_axis = np.cross(z_axis,normal_vector,)

        rotation_angle = np.arccos(np.dot(z_axis,normal_vector,))

        test = True
        i = 0

        T = np.eye(4)
        T[0, 3] = center_point[0] + 0.02 * normal_vector[0]
        T[1, 3] = center_point[1] + 0.02 * normal_vector[1]
        T[2, 3] = center_point[2] + 0.02 * normal_vector[2]
        tilt = np.zeros(3)


        while test:

            cone = o3d.geometry.TriangleMesh.create_cone(radius=0.01, height=0.5, resolution=10)
            cone.compute_triangle_normals()
            
            
            # get rotational matrix from diference between normal vector and z axis

            rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(((rotation_axis + tilt) 
                                / np.linalg.norm(rotation_axis + tilt)) * rotation_angle)
            T[:3, :3] = rotation_matrix
            cone.transform(T)  # Transform the cone to the center point and orientation
            

            mesh_test = copy.deepcopy(mesh_test_org)
            mesh_test += cone

            test = mesh_test.is_self_intersecting()


            i += 1
            if i < 3:
               rotation_angle += 0.3
            elif i < 5:
                rotation_angle -= 0.3
                #find smaller of first two components of rotation axis
                min_index = np.argmin(np.abs(rotation_axis[:2]))
                tilt[min_index] = 0.1

            elif i < 7:
                rotation_angle += 0.3
                min_index = np.argmin(np.abs(rotation_axis[:2]))
                tilt[min_index] = -0.1

                
            
            elif i == 7:
                test = False
                #####print("Failed to add cone without self-intersection after 10 attempts.")
                failed.append(cluster_index)
                break

        #print(f"realsense_cones\\cone_num_{max_index + 1 + cluster_index}.ply")
        o3d.io.write_triangle_mesh(f"/home/lab-3d/realsense_cones/cone_num_{max_index + 1 + cluster_index}.ply", cone)
        #scanning_point = [np.asarray(cone.vertices[1]),np.asarray(cone.triangle_normals[0]),T]
        #calculate vector perpendicular to np.asarray(cone.triangle_normals[0]) with most negative z component

        scanning_point = [np.asarray(cone.vertices[1]),np.asarray(cone.triangle_normals[0])]
        yp = 1
        xp = scanning_point[1][0] * yp / scanning_point[1][1]
        zp = - ( (scanning_point[1][0]*xp) + (scanning_point[1][1]*yp) ) / scanning_point[1][2]
        if zp >0:
            zp = -zp
            xp = -xp
            yp = -yp
        length = np.sqrt(xp**2 + yp**2 + zp**2)
        npe = np.array([xp,yp,zp]) / length
        scanning_point.append(npe)
        ####print(np.dot(scanning_point[1],scanning_point[2]))  # Should be close to 0
        scanning_points.append(scanning_point)
        ####print(f'{cluster_index}/{len(cluster_n_triangles)}')
    #
    print(f'{len(failed)}/{len(cluster_n_triangles)} failed')

    return scanning_points

def find_cluster_centerpoint(mesh, triangle_clusters, cluster_index):
    """
    Find the center point of a cluster in the mesh.
    
    Args:
        mesh (o3d.geometry.TriangleMesh): The input mesh.
        triangle_clusters (np.ndarray): Array of cluster indices for each triangle.
        cluster_n_triangles (np.ndarray): Number of triangles in each cluster.
        clster_index (int): Index of the cluster to find the center point for.
    
    Returns:
        np.ndarray: The center point of the specified cluster.
    """
    indices = np.where(np.array(triangle_clusters) == cluster_index)[0]
    triangles = np.asarray(mesh.triangles)[indices]
    vertices = np.asarray(mesh.vertices)
    
    normals = np.asarray(mesh.triangle_normals)[indices]
    #print(normals)
    # Calculate the center point of the cluster

    center_point = np.mean(vertices[triangles[:,0]], axis=0)
    # find averege normal vector of the cluster
    normal_vector = np.mean(normals, axis=0)
    normal_vector = normal_vector / np.linalg.norm(normal_vector) if np.linalg.norm(normal_vector) > 0 else np.array([0, 0, 1])
    return center_point, normal_vector



mesh = o3d.geometry.TriangleMesh()
mesh = o3d.io.read_triangle_mesh("/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/NewImage_v2.glb")
o3d.io.write_triangle_mesh("/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/poskus.ply", mesh)
""""

self_intersect = mesh.get_self_intersecting_triangles()
print(f'Number of self-intersecting triangles: {len(self_intersect)}/ {len(mesh.triangles)} examples: {np.asarray(self_intersect)[:5]}, examples: {np.asarray(mesh.triangles)[:5]}')
while len(self_intersect) > 0:
    # find most common self-intersecting triangle
    unique, counts = np.unique(np.asarray(self_intersect), return_counts=True)

    # find all unique triangels that have the maximum count
    most_common_triangle = []
    for i in range(len(counts)):
        if counts[i] == max(counts):
            most_common_triangle.append(unique[i])

    print(len(unique), max(counts))
    print(len(most_common_triangle))
    #print(f'Removing triangle {most_common_triangle} which is most common self-intersecting triangle')
    #remove self-intersecting triangles with highest count from mesh
    mesh.remove_triangles_by_index(most_common_triangle)
    mesh.remove_unreferenced_vertices()
    self_intersect = mesh.get_self_intersecting_triangles()
    print(f'Number of self-intersecting triangles: {len(self_intersect)}/ {len(mesh.triangles)}')
    mesh.compute_vertex_normals()


o3d.io.write_triangle_mesh("/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/popravljen.ply", mesh)
"""

mesh_deconstruct = copy.deepcopy(mesh)

meshyp = copy.deepcopy(mesh_deconstruct)
meshyp.compute_triangle_normals()  # Ensure normals are computed before filtering
normals = np.asarray(meshyp.triangle_normals)
y_normals = normals[:, 1] > normal_orientation_threshold  # Adjust the threshold as needed
meshyp.triangles = o3d.utility.Vector3iVector(np.asarray(meshyp.triangles)[y_normals])
mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
    mesh_deconstruct.triangles)[~y_normals])


# keep only triangles with normals in the x direction greater than normal_orientation_threshold
meshxp = copy.deepcopy(mesh_deconstruct)
meshxp.compute_triangle_normals()  # Ensure normals are computed before filtering
normals = np.asarray(meshxp.triangle_normals)
x_normals = normals[:, 0] > normal_orientation_threshold  # Adjust the threshold as needed
meshxp.triangles = o3d.utility.Vector3iVector(np.asarray(meshxp.triangles)[x_normals])
mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
    mesh_deconstruct.triangles)[~x_normals])


# keep only triangles with normals in the y direction less than -normal_orientation_threshold
meshym = copy.deepcopy(mesh_deconstruct)
meshym.compute_triangle_normals()  # Ensure normals are computed before filtering
normals = np.asarray(meshym.triangle_normals)
y_neg_normals = normals[:, 1] < -normal_orientation_threshold  # Adjust the threshold as needed
meshym.triangles = o3d.utility.Vector3iVector(np.asarray(meshym.triangles)[y_neg_normals])
mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
    mesh_deconstruct.triangles)[~y_neg_normals])

# keep only triangles with normals in the x direction less than -normal_orientation_threshold
meshxm = copy.deepcopy(mesh_deconstruct)
meshxm.compute_triangle_normals()  # Ensure normals are computed before filtering
normals = np.asarray(meshxm.triangle_normals)
x_neg_normals = normals[:, 0] < -normal_orientation_threshold  # Adjust the threshold as needed
meshxm.triangles = o3d.utility.Vector3iVector(np.asarray(meshxm.triangles)[x_neg_normals])
mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
    mesh_deconstruct.triangles)[~x_neg_normals]) 

#get scanning points from all four meshes
scanning_points = []
if os.path.exists("/home/lab-3d/realsense_cones"):  
        for file in os.listdir("/home/lab-3d/realsense_cones"):
            os.remove(os.path.join("/home/lab-3d/realsense_cones", file))
scanning_points += get_all_scanning_points(mesh_work=meshyp, mesh_test_org=mesh)
scanning_points += get_all_scanning_points(mesh_work=meshxp, mesh_test_org=mesh)
scanning_points += get_all_scanning_points(mesh_work=meshym, mesh_test_org=mesh)
scanning_points += get_all_scanning_points(mesh_work=meshxm, mesh_test_org=mesh)

#remove scanning points that are too close to each other
lst = [l for l in range(len(scanning_points))]
lst1 = copy.deepcopy(lst)
dk = 0 
for j in lst:
    print(f'j={j}')
    for k in lst1[:j-dk]:
            
        dd = np.linalg.norm(scanning_points[j][0] - scanning_points[k][0])
        da = np.arccos(np.dot(scanning_points[j][1],scanning_points[k][1]))
        if dd < 0.11 and abs(da) < 0.1:
            print(f'Removing {j} because it is too close to {k} (dd={dd}, da={da})')
            lst1.remove(j)
            dk += 1
            break
scanning_points1 = lst1

print(f'Total scanning points: {scanning_points1}, before: {len(lst)}, after: {len(lst1)}')
print(scanning_points)


#o3d.io.write_triangle_mesh(f"realsense_point_cloud_{10}.ply", mesh_long)
o3d.io.write_triangle_mesh(f"realsense_point_cloud_{20}.ply", meshyp)
o3d.io.write_triangle_mesh(f"realsense_point_cloud_{30}.ply", meshxp)
o3d.io.write_triangle_mesh(f"realsense_point_cloud_{40}.ply", meshym)
o3d.io.write_triangle_mesh(f"realsense_point_cloud_{50}.ply", meshxm)
