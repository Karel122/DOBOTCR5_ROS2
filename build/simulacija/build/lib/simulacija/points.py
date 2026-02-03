#!/usr/bin/env python3
import json
import numpy as np
import trimesh
from datetime import date

# ============================================================
# Utility functions
# ============================================================

def normalize(v, eps=1e-12):
    n = np.linalg.norm(v)
    return v if n < eps else v / n


def look_at_rotation(cam_pos, target, up=np.array([0.0, 0.0, 1.0])):
    """
    Returns rotation matrix R (world_from_cam)
    Camera frame convention (for this planner):
      x: right
      y: up
      z: forward (towards target)

    This variant avoids the common 180° roll flip by using:
      x = up x z
      y = z x x
    """
    z = normalize(target - cam_pos)              # forward
    x = normalize(np.cross(up, z))              # right
    if np.linalg.norm(x) < 1e-9:
        x = normalize(np.cross(np.array([0.0, 1.0, 0.0]), z))
    y = np.cross(z, x)                          # up
    return np.column_stack([x, y, z])


def rotation_matrix_to_euler_xyz(R):
    """
    Rotation matrix -> Euler angles (XYZ / roll-pitch-yaw), radians
    Convention matches many robotics "roll-pitch-yaw" descriptions IF you
    interpret it as intrinsic XYZ. (Downstream code can convert if needed.)
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6

    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0

    return rx, ry, rz


def fibonacci_hemisphere(n):
    """
    Uniform-ish directions on +Z hemisphere
    """
    i = np.arange(n)
    phi = (1 + np.sqrt(5)) / 2
    theta = 2 * np.pi * i / phi
    z = (i + 0.5) / n
    r = np.sqrt(1 - z*z)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return np.vstack([x, y, z]).T


# ============================================================
# Geometry checks
# ============================================================

def sample_surface_points(mesh, n_samples=6000):
    pts, face_idx = trimesh.sample.sample_surface(mesh, n_samples)
    normals = mesh.face_normals[face_idx]
    normals = np.array([normalize(n) for n in normals])
    return pts, normals


def distance_mask(points, cam_pos, dmin, dmax):
    d = np.linalg.norm(points - cam_pos[None, :], axis=1)
    return (d >= dmin) & (d <= dmax)


def incidence_mask(points, normals, cam_pos, cos_theta_max):
    """
    Incidence check: keep points whose normal faces the camera within theta_max.
    """
    s = cam_pos[None, :] - points
    s = s / (np.linalg.norm(s, axis=1)[:, None] + 1e-12)
    cosang = np.einsum("ij,ij->i", normals, s)
    return cosang >= cos_theta_max


def fov_mask_camframe(P_cam, tan_ax, tan_ay):
    """
    FOV check in camera coordinates: assumes camera looks along +Z,
    x-right, y-up.
    """
    z = P_cam[:, 2]
    ok = z > 1e-9
    ok &= (np.abs(P_cam[:, 0] / z) <= tan_ax)
    ok &= (np.abs(P_cam[:, 1] / z) <= tan_ay)
    return ok


# ============================================================
# Candidate generation & selection
# ============================================================

def generate_candidates(mesh, n_dirs=300):
    """
    Generate candidate camera poses in the SAME frame as the mesh.
    Here: mesh frame == camera frame (your input condition).
    """
    center = mesh.bounding_box.centroid
    extent = mesh.bounding_box.extents
    radius = 1.2 * 0.5 * np.linalg.norm(extent)

    dirs = fibonacci_hemisphere(n_dirs)
    poses = []

    for d in dirs:
        if d[2] < 0:  # no bottom views (assumes +Z is "up" in this frame)
            continue
        pos = center + radius * d
        R = look_at_rotation(pos, center, up=np.array([0.0, 0.0, 1.0]))
        poses.append((pos, R))

    return poses


def greedy_select_views(cover_sets, k=2, max_views=80):
    """
    Greedy set cover with overlap requirement: each sampled surface point should be
    covered at least k times.
    """
    M = cover_sets[0].shape[0]
    cover_count = np.zeros(M, dtype=int)
    selected = []
    remaining = set(range(len(cover_sets)))

    for _ in range(max_views):
        if np.all(cover_count >= k):
            break

        best_i, best_gain = None, -1
        need = cover_count < k

        for i in remaining:
            gain = int(np.sum(cover_sets[i][need]))
            if gain > best_gain:
                best_gain, best_i = gain, i

        if best_i is None or best_gain <= 0:
            break

        selected.append(best_i)
        cover_count += cover_sets[best_i].astype(int)
        remaining.remove(best_i)

    return selected, cover_count


# ============================================================
# MAIN
# ============================================================

def main():
    # --------- CHANGE THIS PATH ----------
    mesh_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/kamera/mesh/mesh_from_model_aligned.ply"   # mesh is in CAMERA frame
    out_path = "/home/lab-3d/dobot_ws/src/DOBOT_6Axis_ROS2_V4/simulacija/config/points.json"
    # ------------------------------------

    mesh = trimesh.load(mesh_path, force="mesh")
    if mesh.is_empty:
        raise RuntimeError("Mesh is empty or failed to load.")

    # Parameters
    n_surface_samples = 6000
    n_candidate_dirs = 300

    fov_x_deg, fov_y_deg = 70.0, 55.0
    theta_max_deg = 60.0
    dmin, dmax = 0.25, 1.5
    overlap_k = 2
    max_views = 80

    # Sample surface
    pts, normals = sample_surface_points(mesh, n_surface_samples)

    # Remove bottom-facing surface samples (heuristic).
    # IMPORTANT: this assumes +Z in mesh/camera frame is "up" (away from table).
    keep = normals[:, 2] > -0.5
    pts, normals = pts[keep], normals[keep]

    # Precompute FOV and incidence limits
    tan_ax = np.tan(np.deg2rad(fov_x_deg / 2))
    tan_ay = np.tan(np.deg2rad(fov_y_deg / 2))
    cos_theta_max = np.cos(np.deg2rad(theta_max_deg))

    # Candidates in mesh/camera frame
    candidates = generate_candidates(mesh, n_dirs=n_candidate_dirs)

    # Coverage sets
    cover_sets = []
    for pos, R in candidates:
        mask = distance_mask(pts, pos, dmin, dmax)
        mask &= incidence_mask(pts, normals, pos, cos_theta_max)

        if not np.any(mask):
            cover_sets.append(np.zeros(len(pts), bool))
            continue

        # Project masked points into candidate camera frame (R is world_from_cam)
        # cam coords = R^T (world - pos)
        P_cam = (R.T @ (pts[mask] - pos).T).T
        fov_ok = fov_mask_camframe(P_cam, tan_ax, tan_ay)

        full_mask = np.zeros(len(pts), bool)
        full_mask[np.where(mask)[0][fov_ok]] = True
        cover_sets.append(full_mask)

    selected, coverage = greedy_select_views(cover_sets, k=overlap_k, max_views=max_views)

    # OUTPUT: JSON + degrees
    # points: [x, y, z, rx_deg, ry_deg, rz_deg] in CAMERA/MESH FRAME
    poses_2d_deg = []
    for idx in selected:
        pos, R = candidates[idx]
        rx, ry, rz = rotation_matrix_to_euler_xyz(R)      # radians
        rx, ry, rz = np.degrees([rx, ry, rz])             # degrees
        poses_2d_deg.append([
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(rx), float(ry), float(rz)
        ])

    output = {
        "trajectory_name": "view_poses_in_mesh_camera_frame",
        "metadata": {
            "created": str(date.today()),
            "frame_id": "mesh_camera_frame",
            "description": (
                "Geometric view planning in the SAME coordinate frame as the input mesh "
                "(assumed to be the camera frame at capture time). "
                "Points are [x,y,z,rx,ry,rz] with angles in DEGREES; Euler XYZ."
            ),
            "fov_x_deg": float(fov_x_deg),
            "fov_y_deg": float(fov_y_deg),
            "theta_max_deg": float(theta_max_deg),
            "dmin": float(dmin),
            "dmax": float(dmax),
            "overlap_k": int(overlap_k),
            "n_surface_samples": int(n_surface_samples),
            "n_candidate_dirs": int(n_candidate_dirs),
            "max_views": int(max_views),
            "coverage_stats": {
                "selected_views": int(len(poses_2d_deg)),
                "min_coverage": int(np.min(coverage)) if len(coverage) else 0,
                "mean_coverage": float(np.mean(coverage)) if len(coverage) else 0.0,
                "pct_at_least_k": float(np.mean(coverage >= overlap_k) * 100.0) if len(coverage) else 0.0
            }
        },
        "points": poses_2d_deg
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)

    # Terminal summary
    print(f"\nWrote JSON: {out_path}")
    print("selected views:", len(poses_2d_deg))
    print("min coverage:", int(np.min(coverage)))
    print("mean coverage:", float(np.mean(coverage)))
    print("\nFirst 5 points:")
    for p in poses_2d_deg[:5]:
        print([round(v, 6) for v in p])


if __name__ == "__main__":
    main()


