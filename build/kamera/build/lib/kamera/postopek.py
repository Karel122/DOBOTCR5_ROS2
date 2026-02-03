import numpy as np
import open3d as o3d

def remove_table_keep_object(
    pcd: o3d.geometry.PointCloud,
    plane_dist_thresh: float = 0.004,   # ~4 mm (tune to your sensor noise)
    object_height_min: float = 0.008,   # keep points at least 8 mm above plane
    voxel: float = 0.005,
    dbscan_eps: float = 0.02,
    dbscan_min_points: int = 50,
    min_cluster_size_ratio: float = 0.1
):
    if voxel and voxel > 0:
        pcd = pcd.voxel_down_sample(voxel)

    # 1) Fit table plane ax + by + cz + d = 0
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=plane_dist_thresh,
        ransac_n=3,
        num_iterations=3000
    )
    a, b, c, d = plane_model
    n = np.array([a, b, c], dtype=float)
    n_norm = np.linalg.norm(n) + 1e-12
    n_unit = n / n_norm

    pts = np.asarray(pcd.points)

    # 2) Signed distance to plane (positive on one side)
    signed = (pts @ n + d) / n_norm

    # Ensure "above" means away from the camera-table side is consistent:
    # If the camera is above the table, the camera origin is typically above the plane.
    # We choose the sign so the camera is on the positive side.
    cam_side = (np.array([0.0, 0.0, 0.0]) @ n + d) / n_norm
    if cam_side < 0:
        signed = -signed

    # 3) Remove table: drop points near plane and keep points above plane by object_height_min
    keep = signed > object_height_min
    obj = pcd.select_by_index(np.where(keep)[0])

    if obj.is_empty():
        return obj, plane_model

    # 4) Keep main cluster(s)
    labels = np.array(obj.cluster_dbscan(eps=dbscan_eps, min_points=dbscan_min_points, print_progress=False))
    valid = labels >= 0
    if not np.any(valid):
        return obj, plane_model

    unique, counts = np.unique(labels[valid], return_counts=True)
    largest = counts.max()
    keep_clusters = set(unique[counts >= min_cluster_size_ratio * largest])
    keep_idx = np.where(np.array([lbl in keep_clusters for lbl in labels]))[0]

    obj = obj.select_by_index(keep_idx)
    return obj, plane_model


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("pc_cam.ply")
    obj, plane = remove_table_keep_object(
        pcd,
        plane_dist_thresh=0.004,
        object_height_min=0.01,      # 1 cm above table (tune)
        voxel=0.005,
        dbscan_eps=0.02,
        dbscan_min_points=50,
        min_cluster_size_ratio=0.1
    )

    o3d.io.write_point_cloud("object_only.ply", obj)
    print("Saved object_only.ply")
    # o3d.visualization.draw_geometries([obj])
