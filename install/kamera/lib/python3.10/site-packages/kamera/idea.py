import numpy as np
import open3d as o3d


REF_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_depth.ply"
MOV_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/image0ply.ply"
OUT_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_model_aligned.ply"


N_SAMPLE_POINTS = 40000          
VOXEL_FRAC = 0.02                
RANSAC_MAX_ITERS = 200000
RANSAC_CONFIDENCE = 0.999
COARSE_ICP_MULT = 3.0
FINE_ICP_MULT = 1.5
COARSE_ITERS = 40
FINE_ITERS = 60




def load_mesh(path: str) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(path, enable_post_processing=True)
    if mesh is None or len(mesh.vertices) == 0:
        raise ValueError(f"Failed to load a valid mesh from: {path}")
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    return mesh


def mesh_to_pcd_fast(mesh: o3d.geometry.TriangleMesh, n_points: int) -> o3d.geometry.PointCloud:
    return mesh.sample_points_uniformly(number_of_points=n_points)


def aabb_diag(geom) -> float:
    ext = geom.get_axis_aligned_bounding_box().get_extent()
    return float(np.linalg.norm(ext))


def preprocess_for_features(pcd: o3d.geometry.PointCloud, voxel: float):
    p = pcd.voxel_down_sample(voxel)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=60))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        p, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=120)
    )
    return p, fpfh


def run_global_ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel: float):
    dist = 1.5 * voxel
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down,
        src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            RANSAC_MAX_ITERS, RANSAC_CONFIDENCE
        ),
    )
    return result


def icp_point_to_plane(src, tgt, init_T, max_corr: float, iters: int):
    return o3d.pipelines.registration.registration_icp(
        src, tgt, max_corr, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iters),
    )


def main():
    ref_mesh = load_mesh(REF_PATH)
    mov_mesh = load_mesh(MOV_PATH)


    ref_pcd = mesh_to_pcd_fast(ref_mesh, N_SAMPLE_POINTS)
    mov_pcd = mesh_to_pcd_fast(mov_mesh, N_SAMPLE_POINTS)

    
    d_ref = aabb_diag(ref_pcd)
    voxel = max(VOXEL_FRAC * d_ref, 1e-6)
    print(f"[info] ref bbox diag={d_ref:.6f}  voxel={voxel:.6f}")
    print("[info] NOTE: scaling is OFF. If your units differ, fix units upstream or add explicit scale factor.")

    ref_down, ref_fpfh = preprocess_for_features(ref_pcd, voxel)
    mov_down, mov_fpfh = preprocess_for_features(mov_pcd, voxel)

    ransac = run_global_ransac(mov_down, ref_down, mov_fpfh, ref_fpfh, voxel)
    print(f"[ransac] fitness={ransac.fitness:.6f}  rmse={ransac.inlier_rmse:.6f}")
    print("[ransac] T=\n", ransac.transformation)

    max_corr_coarse = COARSE_ICP_MULT * voxel
    icp1 = icp_point_to_plane(mov_down, ref_down, ransac.transformation, max_corr_coarse, COARSE_ITERS)
    print(f"[icp coarse] fitness={icp1.fitness:.6f}  rmse={icp1.inlier_rmse:.6f}")

    voxel_fine = max(0.5 * voxel, 1e-6)
    ref_down2, _ = preprocess_for_features(ref_pcd, voxel_fine)
    mov_down2, _ = preprocess_for_features(mov_pcd, voxel_fine)

    max_corr_fine = FINE_ICP_MULT * voxel_fine
    icp2 = icp_point_to_plane(mov_down2, ref_down2, icp1.transformation, max_corr_fine, FINE_ITERS)
    print(f"[icp fine]   fitness={icp2.fitness:.6f}  rmse={icp2.inlier_rmse:.6f}")
    print("[icp fine] T=\n", icp2.transformation)

    mov_mesh.transform(icp2.transformation)

    ok = o3d.io.write_triangle_mesh(OUT_PATH, mov_mesh, write_ascii=False)
    if not ok:
        raise RuntimeError(f"Failed to write output to: {OUT_PATH}")
    print(f"[done] wrote: {OUT_PATH}")


if __name__ == "__main__":
    main()
