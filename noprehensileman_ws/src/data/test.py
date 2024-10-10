import pandas as pd
import matplotlib.pyplot as plt

# Load the data
data = pd.read_csv("trajectory_data.csv")

# Plot Pose Data
plt.figure(figsize=(10, 8))
for col in ['Pose_X', 'Pose_Y', 'Pose_Z', 'Pose_Roll', 'Pose_Pitch', 'Pose_Yaw']:
    plt.plot(data['Time'], data[col], label=col)
plt.title('Pose over Time')
plt.xlabel('Time [s]')
plt.ylabel('Pose')
plt.legend()
plt.grid(True)
plt.show()

# Plot Velocity Data
plt.figure(figsize=(10, 8))
for col in ['Vel_X', 'Vel_Y', 'Vel_Z', 'Vel_Roll', 'Vel_Pitch', 'Vel_Yaw']:
    plt.plot(data['Time'], data[col], label=col)
plt.title('Velocity over Time')
plt.xlabel('Time [s]')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)
plt.show()

# Plot Acceleration Data
plt.figure(figsize=(10, 8))
for col in ['Acc_X', 'Acc_Y', 'Acc_Z', 'Acc_Roll', 'Acc_Pitch', 'Acc_Yaw']:
    plt.plot(data['Time'], data[col], label=col)
plt.title('Acceleration over Time')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration')
plt.legend()
plt.grid(True)
plt.show()

