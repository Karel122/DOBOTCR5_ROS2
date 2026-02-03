import os
import numpy as np
import open3d as o3d
import trimesh


def glb_to_open3d_mesh(glb_path: str) -> o3d.geometry.TriangleMesh:
    if not os.path.isfile(glb_path):
        raise FileNotFoundError(f"GLB not found: {glb_path}")

    # Force scene so node transforms are applied
    scene = trimesh.load(glb_path, force="scene")

    if isinstance(scene, trimesh.Scene):
        if len(scene.geometry) == 0:
            raise RuntimeError("GLB loaded but scene.geometry is empty.")
        tm = trimesh.util.concatenate(list(scene.geometry.values()))
    else:
        tm = scene

    V = np.asarray(tm.vertices, dtype=np.float64)
    F = np.asarray(tm.faces, dtype=np.int32)

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(V),
        o3d.utility.Vector3iVector(F),
    )
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh


if __name__ == "__main__":
    # IMPORTANT: set the correct path
    glb_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/small_size.glb"
    # ^ change if needed

    mesh_f = glb_to_open3d_mesh(glb_path)
    o3d.io.write_triangle_mesh("foundation_mesh.ply", mesh_f)
    print("foundation mesh:", mesh_f)

    pc_f = mesh_f.sample_points_poisson_disk(number_of_points=100000)
    pc_f = pc_f.voxel_down_sample(0.005)
    o3d.io.write_point_cloud("pc_foundation.ply", pc_f)
    print("foundation point cloud:", pc_f)
