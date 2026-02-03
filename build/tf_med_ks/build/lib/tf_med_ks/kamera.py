import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import scipy.spatial
import copy
import time
import os
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import SetFloat

class KameraNode(Node):
    def __init__(self):
        super().__init__('kamera_node')
        self.publisher = self.create_service(SetFloat, 'tocke_kamera',10)
        #self.time = self.create_timer(1.0,self.publish_camera_points)

        self.d = 0.45  # Min distance threshold for filtering depth data [meters]
        self.dd = 0.23  # Max - Min distance threshold for filtering depth data [meters]
        self.min_cluster_size_ratio = 0.081  # Ratio for minimum cluster size to the largest cluster size
        self.normal_orientation_threshold = 0.3  # Threshold for normal orientation filtering
        self.kamera()






    def remove_small_clusters(self,mesh, min_cluster_size_ratio=0.1):
        """    
        Remove small clusters from the mesh based on the minimum cluster size ratio.
        Args:       
            mesh (o3d.geometry.TriangleMesh): The input mesh.           
            min_cluster_size_ratio (float): The ratio of the minimum cluster size to the largest cluster size.
        Returns:    
            o3d.geometry.TriangleMesh: The mesh with small clusters removed.    
        """
        
        triangle_clusters, cluster_n_triangles, cluster_area = (
            mesh.cluster_connected_triangles())

        # remove clusters smaller than mesh, min_cluster_size_ratio of the largest cluster
        largest_cluster_size = max(cluster_n_triangles)
        min_cluster_size = largest_cluster_size * min_cluster_size_ratio
        clusters_to_remove = np.where(np.array(cluster_n_triangles) <= min_cluster_size,True,False)
        triangles_to_keep = np.where(clusters_to_remove[triangle_clusters] == True,True,False)

        mesh.remove_triangles_by_mask(triangles_to_keep)
        mesh.remove_unreferenced_vertices()  # Clean up the mesh after removing triangles
        mesh.compute_vertex_normals()  # Recompute normals after filtering triangles
        return mesh

    def find_cluster_centerpoint(self,mesh, triangle_clusters, cluster_index):
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

    def get_all_scanning_points(self,mesh_work, mesh_test_org):
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
        if os.path.exists("realsense_cones"):  
            for file in os.listdir("realsense_cones"):
                os.remove(os.path.join("realsense_cones", file))
        

        for cluster_index in range(len(cluster_n_triangles)):
            
            mesh_work.compute_triangle_normals()  # Ensure normals are computed before filtering
            center_point, normal_vector = self.find_cluster_centerpoint(mesh_work, triangle_clusters, cluster_index=cluster_index)

            z_axis = np.array([0, 0, 1])
            rotation_axis = np.cross(z_axis,normal_vector,)

            rotation_angle = np.arccos(np.dot(z_axis,normal_vector,))

            test = True
            i = 0

            T = np.eye(4)
            T[0, 3] = center_point[0] + 0.01 * normal_vector[0]
            T[1, 3] = center_point[1] + 0.01 * normal_vector[1]
            T[2, 3] = center_point[2] + 0.01 * normal_vector[2]
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
                    print("Failed to add cone without self-intersection after 10 attempts.")
                    failed.append(cluster_index)
                    break

            o3d.io.write_triangle_mesh(f"realsense_cones\\cone_num_{cluster_index}.ply", cone)
            scanning_point = [np.asarray(cone.vertices[1]),np.asarray(cone.triangle_normals[0]),T]
            #print(scanning_point)
            scanning_points.append(scanning_point)
            print(f'{cluster_index}/{len(cluster_n_triangles)}')
        #
        print(f'{len(failed)}/{len(cluster_n_triangles)} failed')

        return scanning_points


#use Intel realSense D435 to take rgbd images with region of interest
    def kamera(self):
        pipe = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)

        pipe.start(config)

        time.sleep(8)  # Allow some time for the camera to warm up

        threshold_filter = rs.threshold_filter()
        threshold_filter.set_option(rs.option.min_distance, self.d)  # Set minimum distance
        threshold_filter.set_option(rs.option.max_distance, self.d + self.dd)  # Set maximum distance



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
        startBBox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-1, -1, 0.3), max_bound=(1, 1, 2))
        mesh = mesh.crop(startBBox)
        mesh.remove_unreferenced_vertices()

        # vis = o3d.visualization.Visualizer()
        # vis.create_window(visible=True)
        # # Call only after creating visualizer window.
        # vis.get_render_option().background_color = [1, 0, 0]
        # vis.add_geometry(mesh)
        # vis.run()


        #visualize the mesh
        #o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)
        # duplicate mesh to filter out triangles with long edges
        mesh_deconstruct = copy.deepcopy(mesh)
        mesh_long = copy.deepcopy(mesh_deconstruct)

        #mesh_deconstruct = o3d.geometry.TriangleMesh.create_tetrahedron(1)
        # average the lenght of all the edges of the mesh
        vtx = np.asarray(mesh_deconstruct.vertices)
        edges = np.asarray(mesh_deconstruct.triangles)

        l = [[0,1],[1,2],[2,0]]
        edge_lengths = []
        for j in l:
            edge_lengths += list(np.linalg.norm(vtx[edges[:, j[0]]] 
                                        - vtx[edges[:, j[1]]], axis=1))

        average_edge_length = np.mean(edge_lengths)
        #print(edge_lengths[:5])




        #make copy with only triangels that have long edges
        triangle_edge_lengths = np.zeros(len(edge_lengths)//3)
        for j in range(len(edge_lengths)//3):
            triangle_edge_lengths[j] = edge_lengths[
                j] + edge_lengths[
                j + len(edge_lengths)//3] + edge_lengths[
                j + 2*len(edge_lengths)//3]
            

        long_edges = triangle_edge_lengths > average_edge_length * 5  # Adjust the threshold as needed
        mesh_long.triangles = o3d.utility.Vector3iVector(edges[long_edges])
        mesh_deconstruct.triangles = o3d.utility.Vector3iVector(edges[~long_edges])  # Keep only triangles with short edges
        mesh_long.compute_vertex_normals()  # Recompute normals after filtering triangles
        o3d.io.write_triangle_mesh(f"realsense_point_cloud_{0}.ply", mesh_deconstruct)
        #mesh.compute_vertex_normals()


        # keep only triangels that have normal in y direction grater than 0.5
        meshyp = copy.deepcopy(mesh_deconstruct)
        meshyp.compute_triangle_normals()  # Ensure normals are computed before filtering
        normals = np.asarray(meshyp.triangle_normals)
        y_normals = normals[:, 1] > self.normal_orientation_threshold  # Adjust the threshold as needed
        meshyp.triangles = o3d.utility.Vector3iVector(np.asarray(meshyp.triangles)[y_normals])
        mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
            mesh_deconstruct.triangles)[~y_normals])


        # keep only triangles with normals in the x direction greater than 0.5
        meshxp = copy.deepcopy(mesh_deconstruct)
        meshxp.compute_triangle_normals()  # Ensure normals are computed before filtering
        normals = np.asarray(meshxp.triangle_normals)
        x_normals = normals[:, 0] > self.normal_orientation_threshold  # Adjust the threshold as needed
        meshxp.triangles = o3d.utility.Vector3iVector(np.asarray(meshxp.triangles)[x_normals])
        mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
            mesh_deconstruct.triangles)[~x_normals])


        # keep only triangles with normals in the y direction less than -0.5
        meshym = copy.deepcopy(mesh_deconstruct)
        meshym.compute_triangle_normals()  # Ensure normals are computed before filtering
        normals = np.asarray(meshym.triangle_normals)
        y_neg_normals = normals[:, 1] < -self.normal_orientation_threshold  # Adjust the threshold as needed
        meshym.triangles = o3d.utility.Vector3iVector(np.asarray(meshym.triangles)[y_neg_normals])
        mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
            mesh_deconstruct.triangles)[~y_neg_normals])

        # keep only triangles with normals in the x direction less than -0.5
        meshxm = copy.deepcopy(mesh_deconstruct)
        meshxm.compute_triangle_normals()  # Ensure normals are computed before filtering
        normals = np.asarray(meshxm.triangle_normals)
        x_neg_normals = normals[:, 0] < -self.normal_orientation_threshold  # Adjust the threshold as needed
        meshxm.triangles = o3d.utility.Vector3iVector(np.asarray(meshxm.triangles)[x_neg_normals])
        mesh_deconstruct.triangles = o3d.utility.Vector3iVector(np.asarray(
            mesh_deconstruct.triangles)[~x_neg_normals]) 




        #remove small clusters from mesh
        meshyp = self.remove_small_clusters(meshyp, min_cluster_size_ratio=self.min_cluster_size_ratio)
        meshxp = self.remove_small_clusters(meshxp, min_cluster_size_ratio=self.min_cluster_size_ratio)
        meshym = self.remove_small_clusters(meshym, min_cluster_size_ratio=self.min_cluster_size_ratio)
        meshxm = self.remove_small_clusters(meshxm, min_cluster_size_ratio=self.min_cluster_size_ratio)

        mesh.compute_triangle_normals()
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=2000)  # Simplify the mesh to reduce complexity

        BBox = mesh.get_minimal_oriented_bounding_box()


        # vis = o3d.visualization.Visualizer()
        # vis.create_window(visible=True)
        # # Call only after creating visualizer window.
        # vis.get_render_option().background_color = [1, 0, 0]
        # vis.add_geometry(mesh)
        # vis.add_geometry(BBox)
        # vis.run()
        #############################################################################
        # mesh = o3d.io.read_triangle_mesh("realsense_point_cloud_0.ply")
        # mesh_long = o3d.io.read_triangle_mesh("realsense_point_cloud_10.ply")
        # meshyp = o3d.io.read_triangle_mesh("realsense_point_cloud_20.ply")
        # meshxp = o3d.io.read_triangle_mesh("realsense_point_cloud_30.ply")
        # meshym = o3d.io.read_triangle_mesh("realsense_point_cloud_40.ply")
        # meshxm = o3d.io.read_triangle_mesh("realsense_point_cloud_50.ply")
        ##############################################################################
        scanning_points = []
        scanning_pointsyp = self.get_all_scanning_points(mesh_work=meshyp, mesh_test_org=mesh)
        scanning_pointsxp = self.get_all_scanning_points(mesh_work=meshxp, mesh_test_org=mesh)
        scanning_pointsym = self.get_all_scanning_points(mesh_work=meshym, mesh_test_org=mesh)
        scanning_pointsxm = self.get_all_scanning_points(mesh_work=meshxm, mesh_test_org=mesh)
        scanning_points = [scanning_pointsyp, 
                        scanning_pointsxp, 
                        scanning_pointsym, 
                        scanning_pointsxm]

        for sp in scanning_points:
            lst = [l for l in range(len(sp))]
            lst1 = copy.deepcopy(lst)
            dk = 0 
            for j in lst:
                print(f'j={j}')
                for k in lst1[:j-dk]:
                        
                    self.dd = np.linalg.norm(sp[j][0] - sp[k][0])
                    # reduction[k,j] is equal to vectoral product of scanning_points[j][1] and scanning_points[k][1]
                    da = np.arccos(np.dot(sp[j][1], sp[k][1]))
                    if self.dd < 0.11 and abs(da) < 0.1:
                        print(f'Removing {j} because it is too close to {k} (dd={self.dd}, da={da})')
                        lst1.remove(j)
                        dk += 1
                        break
            lst1

            print(f'Total scanning points: {lst1}, before: {len(lst)}, after: {len(lst1)}')
            spk = []
            for j in lst1:
                spk.append([[sp[j][0]], [sp[j][1]]])
            print(f'Final scanning points: {spk}')
        #remove files from realsense_cones that are not in scanning_points
        # for file in os.listdir("realsense_cones"):
        #     index = int(file.split("_")[-1].split(".")[0])
        #     if index not in scanning_points:
        #         os.remove(os.path.join("realsense_cones", file))





        # #save mesh to file

        # o3d.io.write_triangle_mesh(f"realsense_point_cloud_{10}.ply", mesh_long)
        # o3d.io.write_triangle_mesh(f"realsense_point_cloud_{20}.ply", meshyp)
        # o3d.io.write_triangle_mesh(f"realsense_point_cloud_{30}.ply", meshxp)
        # o3d.io.write_triangle_mesh(f"realsense_point_cloud_{40}.ply", meshym)
        # o3d.io.write_triangle_mesh(f"realsense_point_cloud_{50}.ply", meshxm)

def main(args=None):
    rclpy.init(args=args)
    node = KameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
