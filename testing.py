import pyzed.sl as sl
import pyvista as pv
import numpy as np
import open3d as o3d

def main():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_maximum_distance = 2

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Error: Camera could not be opened.")
        return

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.enable_area_memory = True
    if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
        print("Error: Positional tracking failed.")
        zed.close()
        return

    runtime = sl.RuntimeParameters()
    point_cloud = sl.Mat()

    # Initialize Open3D global point cloud container
    global_pcd = o3d.geometry.PointCloud()
    voxel_size = 0.05  # 2cm voxel resolution to prevent duplicate points

    plotter = pv.Plotter()
    plotter.show(auto_close=False, interactive_update=True)
    
    print("Running room SLAM accumulation. Move the camera around the room.")
    print("Close the window to stop and view the final accumulated map.")
    
    frame_counter = 0
    while plotter.render_window.GetInteractor().GetDone() == 0:
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            pc_np = point_cloud.get_data()
            
            points = pc_np[:, :, :3].reshape(-1, 3)
            colors_raw = pc_np[:, :, 3].reshape(-1)
            
            mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
            valid_points = points[mask]
            valid_colors_raw = colors_raw[mask]
            
            if len(valid_points) > 0:
                # Downsample current frame before adding to save processing time
                valid_points = valid_points[::4]
                valid_colors_raw = valid_colors_raw[::4]
                
                colors_rgba = valid_colors_raw.view(np.uint32)
                r = (colors_rgba & 0x000000FF).astype(np.float64) / 255.0
                g = ((colors_rgba & 0x0000FF00) >> 8).astype(np.float64) / 255.0
                b = ((colors_rgba & 0x00FF0000) >> 16).astype(np.float64) / 255.0
                rgb = np.stack([r, g, b], axis=-1)
                
                # Create temporary Open3D point cloud for the current frame
                current_pcd = o3d.geometry.PointCloud()
                current_pcd.points = o3d.utility.Vector3dVector(valid_points)
                current_pcd.colors = o3d.utility.Vector3dVector(rgb)
                
                # Retrieve camera pose and transform into the world frame correctly
                pose = sl.Pose()
                if zed.get_position(pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
                    # Fix: call pose_data() as a function to access the 4x4 numpy matrix .m
                    transform_matrix = pose.pose_data().m
                    current_pcd.transform(transform_matrix)

                    print(pose.pose_data())
                    
                    # Accumulate into global point cloud and downsample via voxel grid
                    global_pcd += current_pcd
                    global_pcd = global_pcd.voxel_down_sample(voxel_size=voxel_size)
                
                # Update PyVista visualization every 5 frames for smooth rendering performance
                frame_counter += 1
                if frame_counter % 5 == 0:
                    pts_np = np.asarray(global_pcd.points)
                    cols_np = (np.asarray(global_pcd.colors) * 255).astype(np.uint8)
                    
                    if len(pts_np) > 0:
                        plotter.clear()
                        cloud = pv.PolyData(pts_np)
                        cloud.point_data["colors"] = cols_np
                        plotter.add_mesh(cloud, point_size=10, rgb=True)
        
        plotter.update()
        
    zed.close()
    
    # Save the final accumulated room scan to disk on exit
    o3d.io.write_point_cloud("room_scan.ply", global_pcd)
    print("Saved accumulated room point cloud to room_scan.ply")

if __name__ == "__main__":
    main()