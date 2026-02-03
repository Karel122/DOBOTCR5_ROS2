import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d


REF_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_depth.ply"
MOV_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/image0ply.ply"
OUT_PATH = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_model_aligned.ply"


class AllignModels(Node):
    def __init__(self):
        super().__init__('align_models')
        

        # -----------------------------
        # Sampling + registration params
        # -----------------------------

        self.N_SAMPLE_POINTS = 40000
        self.VOXEL_FRAC = 0.02
        self.RANSAC_MAX_ITERS = 200000
        self.RANSAC_CONFIDENCE = 0.999

        self.COARSE_ICP_MULT = 3.0
        self.FINE_ICP_MULT = 1.5
        self.COARSE_ITERS = 40
        self.FINE_ITERS = 60

        # -----------------------------
        # Multi-try + quality gate params
        # -----------------------------
        self.TRIES_PER_EPOCH = 5
        self.MAX_EPOCHS = 3

        ###############  We can Tune these:#############

        self.GOOD_MIN_FITNESS = 0.70
        self.GOOD_MAX_RMSE = 0.010

        self.FALLBACK_TO_BEST = True

        # Reproducibility (set int like 42, or None)
        self.BASE_SEED = 42




        ref_mesh = self.load_mesh(REF_PATH)
        mov_mesh = self.load_mesh(MOV_PATH)

        print("[info] NOTE: scaling is OFF. If your units differ, fix units upstream or add explicit scale factor.")
        print(f"[gate] GOOD_MIN_FITNESS={self.GOOD_MIN_FITNESS}  GOOD_MAX_RMSE={self.GOOD_MAX_RMSE}")
        print(f"[retry] TRIES_PER_EPOCH={self.TRIES_PER_EPOCH}  MAX_EPOCHS={self.MAX_EPOCHS}  FALLBACK_TO_BEST={self.FALLBACK_TO_BEST}")
        print(f"[refine] Using Generalized ICP? {self.has_generalized_icp()} (will fall back to point-to-plane ICP if False)")

        best_overall = None
        best_good = None

        for epoch in range(1, self.MAX_EPOCHS + 1):
            print(f"\n========== EPOCH {epoch}/{self.MAX_EPOCHS} ==========")
            found_good = False

            for t in range(1, self.TRIES_PER_EPOCH + 1):
                seed = None
                if self.BASE_SEED is not None:
                    seed = self.BASE_SEED + (epoch * 1000) + t

                res = self.single_attempt(ref_mesh, mov_mesh, seed=seed)

                print(
                    f"[try {t}/{self.TRIES_PER_EPOCH}] "
                    f"voxel={res['voxel']:.6f} fine={res['voxel_fine']:.6f} | "
                    f"REFINE fitness={res['refine_fitness']:.6f} rmse={res['refine_rmse']:.6f} | "
                    f"RANSAC fitness={res['ransac_fitness']:.6f} rmse={res['ransac_rmse']:.6f}"
                )

                best_overall = self.pick_better(best_overall, res)

                if self.is_good(res["refine_fitness"], res["refine_rmse"]):
                    best_good = self.pick_better(best_good, res)
                    found_good = True
                    print(" [gate] Good alignment found in this try.")
                    # If we want, stop early:
                    break

            if found_good:
                print("✅ [epoch] Found at least one good alignment. Stopping retries.")
                break
            else:
                print(" [epoch] No good alignment found in this epoch.")

        chosen = best_good if best_good is not None else best_overall

        if chosen is None:
            raise RuntimeError("No alignment results produced.")

        if best_good is None and not self.FALLBACK_TO_BEST:
            raise RuntimeError(
                "No alignment passed the quality gate, and FALLBACK_TO_BEST=False. "
                "Tune GOOD_MIN_FITNESS / GOOD_MAX_RMSE or improve preprocessing."
            )

        print("\n========== CHOSEN RESULT ==========")
        print(f"[chosen] REFINE fitness={chosen['refine_fitness']:.6f}  rmse={chosen['refine_rmse']:.6f}")
        print(f"[chosen] Used GICP: {chosen['used_gicp']}")
        print("[chosen] T=\n", chosen["T"])

        # Apply transform to a FRESH moving mesh to avoid cumulative transforms
        mov_mesh_out = self.load_mesh(MOV_PATH)
        mov_mesh_out.transform(chosen["T"])

        ok = o3d.io.write_triangle_mesh(OUT_PATH, mov_mesh_out, write_ascii=False)
        if not ok:
            raise RuntimeError(f"Failed to write output to: {OUT_PATH}")
        print(f"[done] wrote: {OUT_PATH}")





        
    def load_mesh(self,path: str) -> o3d.geometry.TriangleMesh:
        mesh = o3d.io.read_triangle_mesh(path, enable_post_processing=True)
        if mesh is None or len(mesh.vertices) == 0:
            raise ValueError(f"Failed to load a valid mesh from: {path}")
        mesh.remove_duplicated_vertices()
        mesh.remove_duplicated_triangles()
        mesh.remove_degenerate_triangles()
        mesh.remove_non_manifold_edges()
        mesh.compute_vertex_normals()
        return mesh


    def mesh_to_pcd(self, mesh: o3d.geometry.TriangleMesh, n_points: int) -> o3d.geometry.PointCloud:
            return mesh.sample_points_poisson_disk(number_of_points=n_points)


    def aabb_diag(self, geom) -> float:
        ext = geom.get_axis_aligned_bounding_box().get_extent()
        return float(np.linalg.norm(ext))


    def preprocess_for_features(self, pcd: o3d.geometry.PointCloud, voxel: float):
        p = pcd.voxel_down_sample(voxel)
        p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=60))
        fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            p, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=120)
        )
        return p, fpfh


    def run_global_ransac(self, src_down, tgt_down, src_fpfh, tgt_fpfh, voxel: float):
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
                self.RANSAC_MAX_ITERS, self.RANSAC_CONFIDENCE
            ),
        )
        return result


    def has_generalized_icp(self):
        return hasattr(o3d.pipelines.registration, "registration_generalized_icp")


    def gicp(self, src, tgt, init_T, max_corr: float, iters: int):
        """
        Generalized ICP refinement.
        Falls back to point-to-plane ICP if not available.
        """
        # if has_generalized_icp():
        #     return o3d.pipelines.registration.registration_generalized_icp(
        #         src, tgt, max_corr, init_T,
        #         o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        #         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iters),
        #     )
        # """
        # # Fallback (keeps script working on older Open3D builds)
        return o3d.pipelines.registration.registration_icp(
            src, tgt, max_corr, init_T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iters),
        )


    def is_good(self, fitness: float, rmse: float) -> bool:
        return (fitness >= self.GOOD_MIN_FITNESS) and (rmse <= self.GOOD_MAX_RMSE)


    def single_attempt(self, ref_mesh, mov_mesh, seed=None):
        if seed is not None:
            np.random.seed(seed)

        # Sample meshes -> point clouds
        ref_pcd = self.mesh_to_pcd(ref_mesh, self.N_SAMPLE_POINTS)
        mov_pcd = self.mesh_to_pcd(mov_mesh, self.N_SAMPLE_POINTS)

        # Voxel size tied to reference scale
        d_ref = self.aabb_diag(ref_pcd)
        voxel = max(self.VOXEL_FRAC * d_ref, 1e-6)

        # Downsample + features for global registration
        ref_down, ref_fpfh = self.preprocess_for_features(ref_pcd, voxel)
        mov_down, mov_fpfh = self.preprocess_for_features(mov_pcd, voxel)

        # Global init
        ransac = self.run_global_ransac(mov_down, ref_down, mov_fpfh, ref_fpfh, voxel)

        # COARSE refinement (GICP)
        max_corr_coarse = self.COARSE_ICP_MULT * voxel
        reg1 = self.gicp(mov_down, ref_down, ransac.transformation, max_corr_coarse, self.COARSE_ITERS)

        # FINE refinement (GICP) on denser clouds
        voxel_fine = max(0.5 * voxel, 1e-6)
        ref_down2, _ = self.preprocess_for_features(ref_pcd, voxel_fine)
        mov_down2, _ = self.preprocess_for_features(mov_pcd, voxel_fine)

        max_corr_fine = self.FINE_ICP_MULT * voxel_fine
        reg2 = self.gicp(mov_down2, ref_down2, reg1.transformation, max_corr_fine, self.FINE_ITERS)

        return {
            "voxel": voxel,
            "voxel_fine": voxel_fine,
            "ransac_fitness": float(ransac.fitness),
            "ransac_rmse": float(ransac.inlier_rmse),
            "refine_fitness": float(reg2.fitness),
            "refine_rmse": float(reg2.inlier_rmse),
            "T": reg2.transformation,
            "used_gicp": self.has_generalized_icp(),
        }


    def pick_better(self, a, b):
        if a is None:
            return b
        if b is None:
            return a
        if b["refine_fitness"] > a["refine_fitness"] + 1e-9:
            return b
        if abs(b["refine_fitness"] - a["refine_fitness"]) <= 1e-9 and b["refine_rmse"] < a["refine_rmse"]:
            return b
        return a


def main():
    rclpy.init()
    node = AllignModels()
    node.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
