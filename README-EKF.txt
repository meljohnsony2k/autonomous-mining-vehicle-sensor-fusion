## ✅ Summary of Updates (April 2025)

### 1. EKF Sensor Fusion Configuration (`ekf.yaml`)
- Sensor inputs:
  - `/imu`: IMU data (orientation + covariance)
  - `/amcl_pose`: Position estimate from AMCL (LiDAR + map)
  - `/filtered_navsat/fix`: Filtered GPS data combined with IMU via `navsat_transform_node` → `/odometry/gps`
- Output:
  - `/odometry/filtered`: Final EKF fused pose (TF publishing is disabled to avoid conflicts)

📁 Path:
src/your_localization_pkg/config/ekf.yaml

### 2. Launch File (`localization.launch`)
- Starts:
  - `ekf_localization_node`
  - `navsat_transform_node`
- `use_sim_time=true` is set for Gazebo compatibility
- TF Management:
  - EKF does **not** publish TFs (`publish_tf: false`) to avoid conflicts with Gazebo and AMCL
  - TF tree remains clean: `map → odom → base_link`

📁 Path:
src/your_localization_pkg/launch/localization.launch

### 3. RViz Visualization (`localization_visualization_clean.rviz`)
- Visualized topics:
  - `/odometry/filtered` (green): EKF output
  - `/amcl_pose` (blue): LiDAR-based localization
  - `/odometry/gps` (red): GPS pose estimate
  - `/tf`: Coordinate frames
- All displays are cleaned and optimized for clarity
- `Fixed Frame`: `map`

📁 Path:
src/your_localization_pkg/rviz/localization_visualization_clean.rviz


### 4. Trajectory Recording (Optional)
You can record all trajectory topics to a ROS bag for replay or analysis:

```bash
rosbag record -O trajectory.bag /odometry/filtered /amcl_pose /odometry/gps
