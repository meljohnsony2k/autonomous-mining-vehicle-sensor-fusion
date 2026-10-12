# Import
import numpy as np
import pandas as pd
import pymap3d as pm
import plotly.express as px
from scipy.signal import butter, filtfilt
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import time

# Load the data into pandas DataFrames
accel_data = pd.read_csv('slack2/Linear Acceleration.csv', delimiter=',')
gyro_data = pd.read_csv('slack2/Gyroscope.csv', delimiter=',')
gps_data = pd.read_csv('slack2/Location.csv', delimiter=',')

############# IMU ################

# Clean CSV headers
gps_data.columns = gps_data.columns.str.replace('"', '').str.strip()
accel_data.columns = accel_data.columns.str.replace('"', '').str.strip()
gyro_data.columns = gyro_data.columns.str.replace('"', '').str.strip()

# Convert time to seconds (assuming millisecond timestamps)
accel_data["Time (s)"] -= accel_data["Time (s)"].iloc[0]
gyro_data["Time (s)"] -= gyro_data["Time (s)"].iloc[0]

# Lowpass filter (Noise Reduction)
def butter_lowpass_filter(data, cutoff=1, fs=10, order=3):
    nyquist = 0.5 * fs  # Nyquist frequency
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Apply the corrected filter
for axis in ["Linear Acceleration x (m/s^2)", "Linear Acceleration y (m/s^2)", "Linear Acceleration z (m/s^2)"]:
    accel_data[axis] = butter_lowpass_filter(accel_data[axis])

for axis in ["Gyroscope x (rad/s)", "Gyroscope y (rad/s)", "Gyroscope z (rad/s)"]:
    gyro_data[axis] = butter_lowpass_filter(gyro_data[axis])

# Visualize Raw IMU Data
'''fig = px.line(accel_data, x="Time (s)", y=["Linear Acceleration x (m/s^2)",
                                           "Linear Acceleration y (m/s^2)",
                                           "Linear Acceleration z (m/s^2)"],
              title="Accelerometer Readings", labels={"value": "Acceleration (m/s²)"})
fig.show()

fig = px.line(gyro_data, x="Time (s)", y=["Gyroscope x (rad/s)",
                                          "Gyroscope y (rad/s)",
                                          "Gyroscope z (rad/s)"],
              title="Gyroscope Readings", labels={"value": "Angular Velocity (rad/s)"})
fig.show()'''

# Compute Orientation
# Integrate gyroscope data to estimate orientation
dt=0.1 # 10 Hz sensor update rate
gyro_data["Roll (°)"] = np.cumsum(gyro_data["Gyroscope x (rad/s)"]) * dt * 180 / np.pi
gyro_data["Pitch (°)"] = np.cumsum(gyro_data["Gyroscope y (rad/s)"]) * dt * 180 / np.pi
gyro_data["Yaw (°)"] = np.cumsum(gyro_data["Gyroscope z (rad/s)"]) * dt * 180 / np.pi

# Visualizing Orientation
'''fig = px.line(gyro_data, x="Time (s)", y=["Roll (°)", "Pitch (°)", "Yaw (°)"],
           title="Estimated IMU Orientation (Roll, Pitch, Yaw)", labels={"value": "Angle (°)"})
fig.show()'''

# Compute IMU-Based Position (Double Integration)
# Integrate acceleration to get velocity
accel_data["Vx (m/s)"] = np.cumsum(accel_data["Linear Acceleration x (m/s^2)"]) * dt
accel_data["Vy (m/s)"] = np.cumsum(accel_data["Linear Acceleration y (m/s^2)"]) * dt
accel_data["Vz (m/s)"] = np.cumsum(accel_data["Linear Acceleration z (m/s^2)"]) * dt
# Plot velocity (Vx, Vy, Vz)
'''fig = px.line(accel_data, x="Time (s)", y=["Vx (m/s)", "Vy (m/s)", "Vz (m/s)"],
              title="IMU Estimated Velocity Over Time", labels={"value": "Velocity (m/s)"})
fig.show()'''

# Integrate velocity to get position
accel_data["Px (m)"] = np.cumsum(accel_data["Vx (m/s)"]) * dt
accel_data["Py (m)"] = np.cumsum(accel_data["Vy (m/s)"]) * dt
accel_data["Pz (m)"] = np.cumsum(accel_data["Vz (m/s)"]) * dt
# Plot position (Px, Py, Pz)
'''fig = px.line(accel_data, x="Time (s)", y=["Px (m)", "Py (m)", "Pz (m)"],
              title="IMU Estimated Position Over Time", labels={"value": "Position (m)"})
fig.show()'''

# Visualization of IMU Trajectory
# Compute Yaw in Radians
gyro_data["Yaw (rad)"] = np.cumsum(gyro_data["Gyroscope z (rad/s)"]) * dt
# Estimate velocity from IMU acceleration
accel_data["Speed (m/s)"] = np.sqrt(accel_data["Linear Acceleration x (m/s^2)"]**2 + accel_data["Linear Acceleration y (m/s^2)"]**2)

# Initialize trajectory arrays
x, y = [0], [0]  # Start at (0,0)
v= [0]  # Start velocity at 0
yaw = [0]

# Compute trajectory
for i in range(1, len(gyro_data)):
    psi = gyro_data["Yaw (rad)"].iloc[i]  # Get yaw angle
    v_new = accel_data["Speed (m/s)"].iloc[i]  # Get speed
    # Compute displacement using normalized speed
    dx = v_new * np.cos(psi)
    dy = v_new * np.sin(psi)
    # Update position and velocity
    x.append(x[-1] + dx)
    y.append(y[-1] + dy)
    v.append(v_new)
    yaw.append(psi)

# Create DataFrame for trajectory
imu_df = pd.DataFrame({"X (m)": x, "Y (m)": y, "V (m/s)": v, "Yaw (rad)": yaw, "Time (s)": gyro_data["Time (s)"]})

# Plot Velocity Over Time
'''fig = px.line(imu_df, x="Time (s)", y="V (m/s)",
              title="Estimated Speed Over Time (IMU-based)",
              labels={"V (m/s)": "Speed (m/s)"})
fig.show()'''

# Plot 2D trajectory
# Apply 90° counterclockwise rotation
imu_df["X_rot"] = -imu_df["Y (m)"]
imu_df["Y_rot"] = imu_df["X (m)"]

# Plot the rotated trajectory
fig = px.line(imu_df, x="X_rot", y="Y_rot",
              title="2D Trajectory Rotated 90° Counterclockwise",
              labels={"X_rot": "Rotated X (m)", "Y_rot": "Rotated Y (m)"})
fig.show()

############# GPS ################

# Define the reference GPS point (usually the first recorded GPS position)
ref_lat = gps_data["Latitude (°)"].iloc[0]
ref_lon = gps_data["Longitude (°)"].iloc[0]
ref_alt = gps_data["Height (m)"].iloc[0]

# Convert GPS (Lat, Lon, Height) to local ENU (East-North-Up)
gps_data["x"], gps_data["y"], gps_data["z"], gps_data["v"] ,gps_data["Time (s)"] = zip(
    *gps_data.apply(lambda row: (
        *pm.geodetic2enu(row["Latitude (°)"], row["Longitude (°)"], row["Height (m)"], ref_lat, ref_lon, ref_alt),
        row["Velocity (m/s)"],  # Keep original speed data
        row['Time (s)']
    ), axis=1))

# Define rotation angle (convert degrees to radians)
theta = np.radians(15)

# Apply 2D rotation transformation
gps_data["x_rot"] = gps_data["x"] * np.cos(theta) - gps_data["y"] * np.sin(theta)
gps_data["y_rot"] = gps_data["x"] * np.sin(theta) + gps_data["y"] * np.cos(theta)

# Plot rotated ENU trajectory
fig = px.line(gps_data, x="x_rot", y="y_rot",
              title="ENU Trajectory Rotated",
              labels={"x_rot": "East (m)", "y_rot": "North (m)"},
              markers=True)  # Adds points to the line

fig.show()

# Fill NaN values in Velocity with 0
gps_data["Velocity (m/s)"] = gps_data["Velocity (m/s)"].fillna(0)

# Plot GPS

# Compute the center of the GPS trajectory for zooming
center_lat = gps_data["Latitude (°)"].mean()
center_lon = gps_data["Longitude (°)"].mean()

fig = px.scatter_map(
    gps_data,
    lat="Latitude (°)", lon="Longitude (°)",
    color="Horizontal Accuracy (m)",
    size="Velocity (m/s)",
    zoom=15,
    center={"lat": center_lat, "lon": center_lon},  # Center the map
    title="GPS Path on Map"
)
fig.show()

################# Data Normalizing #############

# Print confirmation
print(f"IMU DataFrame size : {imu_df.shape}")
print(f"GPS DataFrame size : {gps_data.shape}")

# Compute downsampling factor
downsample_factor = len(imu_df) // len(gps_data)

# Select every Nth row
imu_df_downsampled = imu_df.iloc[::downsample_factor].reset_index(drop=True)

# Ensure exact match in row count
imu_df_downsampled = imu_df_downsampled.iloc[:len(gps_data)]

# Print new sizes
print(f"Downsampled IMU DataFrame size: {imu_df_downsampled.shape}")
print(f"Downsampled GPS DataFrame size: {gps_data.shape}")

############# A-UKF ################
import numpy as np

class AdaptiveUKF:
    def __init__(self, dt):
        self.dt = dt  # Time step

        # Define state dimension (x, y, v)
        self.dim_x = 3
        self.dim_z = 3  

        # Initialize state vector (start at origin, zero velocity)
        self.x = np.array([0, 0, 0])

        # Covariance matrix (initial uncertainty)
        self.P = np.diag([10, 10, 1])

        # Process noise covariance (IMU uncertainty)
        self.Q = np.diag([100, 100, 50])

        # Measurement noise covariance (GPS uncertainty)
        self.R = np.diag([0.1, 0.1, 0.05])

        # State transition matrix
        self.F = np.eye(3)
        self.F[0, 2] = dt  # Update x based on v
        self.F[1, 2] = dt  # Update y based on v

        # Measurement matrix (maps state to measurements)
        self.H = np.eye(3)  # Identity since GPS provides x, y, v

    def predict(self, x, y, v, yaw):
        """EKF Prediction step using IMU speed and estimated heading."""
        self.x[0] = x
        self.x[1] = y
        self.x[2] = v

        # Predict covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, gps_x, gps_y, gps_v, gps_acc):
        """EKF Update step using GPS position and speed."""
        # GPS measurement vector
        z = np.array([gps_x, gps_y, gps_v])

        # Innovation (Residual)
        y = z - np.dot(self.H, self.x)

        # Innovation covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # Kalman gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update state
        self.x = self.x + np.dot(K, y)

        # Update covariance
        I = np.eye(3)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

        # Adaptive noise update: Increase R if residual is large
        error = np.linalg.norm(y)
        if error > 10:
            self.R *= 1.1  # Increase noise assumption if error high
        else:
            self.R *= 0.9  # Reduce noise assumption if error low

import pandas as pd

# Initialize Adaptive EKF
dt = 0.1  # Time step (10 Hz update rate)
aekf = AdaptiveUKF(dt)

# Store results for plotting
x_est, y_est, v_est = [], [], []

# Initialize estimated yaw (starting at 0)
yaw = 0

# Loop through IMU & GPS data
for i in range(len(imu_df_downsampled)):
    # Get IMU speed magnitude
    x = imu_df_downsampled["X_rot"].iloc[i]
    y = imu_df_downsampled["Y_rot"].iloc[i]
    v = imu_df_downsampled["V (m/s)"].iloc[i]
    yaw = imu_df_downsampled["Yaw (rad)"].iloc[i]

    # Predict step using IMU speed and estimated yaw
    aekf.predict(x, y, v, yaw)

    # GPS update (if available)
    if i < len(gps_data):
        gps_x = gps_data["x_rot"].iloc[i]
        gps_y = gps_data["y_rot"].iloc[i]
        gps_v = gps_data["v"].iloc[i]
        gps_acc = gps_data["Horizontal Accuracy (m)"].iloc[i]  # Accuracy in meters

        # Update only if GPS accuracy is good (< 10m)
        if gps_acc < 15:
            aekf.update(gps_x, gps_y, gps_v, gps_acc)

    # Store estimated values
    x_est.append(aekf.x[0])
    y_est.append(aekf.x[1])
    v_est.append(aekf.x[2])

import plotly.express as px

# Convert results to DataFrame
estimated_df = pd.DataFrame({"x": x_est, "y": y_est, "v": v_est})

# Create scatter plot with speed as a heatmap
fig = px.scatter(estimated_df, x="x", y="y", color="v",
                 color_continuous_scale="plasma",
                 title="Adaptive EKF Estimated Path with Speed",
                 labels={"x": "X (m)", "y": "Y (m)", "v": "Speed (m/s)"})

# Add trajectory line
fig.add_trace(px.line(estimated_df, x="x", y="y").data[0])

# Show plot
fig.show()
