import numpy as np
import cv2
import open3d as o3d
import pyrealsense2 as rs


def main():
    # ---------- RealSense pipeline ----------
    pipeline = rs.pipeline()
    config = rs.config()

    # Choose a common stream config (adjust if you want)
    width, height, fps = 640, 480, 30
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

    profile = pipeline.start(config)

    # Align depth to color (so depth matches color pixels)
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Depth scale (meters per depth unit)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale (m per unit): {depth_scale}")

    # Warm up auto-exposure etc.
    for _ in range(30):
        pipeline.wait_for_frames()

    # ---------- Capture one aligned frame ----------
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        pipeline.stop()
        raise RuntimeError("Could not get both depth and color frames.")

    depth_image = np.asanyarray(depth_frame.get_data())  # uint16
    color_image = np.asanyarray(color_frame.get_data())  # uint8 BGR

    # Save raw images (optional)
    cv2.imwrite("color.png", color_image)
    cv2.imwrite("depth.png", depth_image)  # raw z16 depth units

    # ---------- Get intrinsics ----------
    color_intr = color_frame.profile.as_video_stream_profile().intrinsics
    fx, fy, cx, cy = color_intr.fx, color_intr.fy, color_intr.ppx, color_intr.ppy
    w, h = color_intr.width, color_intr.height
    print(f"Intrinsics: fx={fx}, fy={fy}, cx={cx}, cy={cy}, w={w}, h={h}")

    # ---------- Build Open3D RGBD ----------
    # Convert color BGR->RGB for Open3D
    color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    o3d_color = o3d.geometry.Image(color_rgb)
    o3d_depth = o3d.geometry.Image(depth_image)

    intr = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)

    # depth_scale in Open3D is "depth units per meter"
    # RealSense: meters per unit = depth_scale -> units per meter = 1 / depth_scale
    depth_units_per_meter = 1.0 / depth_scale

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color,
        o3d_depth,
        depth_scale=depth_units_per_meter,
        depth_trunc=2.0,                 # meters; adjust to your scene
        convert_rgb_to_intensity=False
    )

    # ---------- Create point cloud in CAMERA frame ----------
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr)

    # Open3D uses a camera convention that often looks "flipped" when visualizing.
    # If you just want to SAVE a camera-frame cloud for robotics math, you may want NO flip.
    # If visualization looks upside-down, uncomment the flip below.
    #
    # pcd.transform([[1, 0, 0, 0],
    #                [0,-1, 0, 0],
    #                [0, 0,-1, 0],
    #                [0, 0, 0, 1]])

    # Optional: remove invalid points (z==0)
    pts = np.asarray(pcd.points)
    valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0)
    pcd = pcd.select_by_index(np.where(valid)[0])

    o3d.io.write_point_cloud("pc_cam.ply", pcd)
    print(pcd)
    print("Saved: pc_cam.ply, color.png, depth.png")

    pipeline.stop()

    # Optional quick view
    # o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()
