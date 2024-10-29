import pandas as pd
import matplotlib.pyplot as plt
import os

# Get the current directory
current_directory = os.path.dirname(os.path.realpath(__file__))

# Set the filenames for the CSV files
arm_pose_file = os.path.join(current_directory, "csv_arm_pose.csv")
joint_positions_file = os.path.join(current_directory, "joint_pos.csv")
control_torques_file = os.path.join(current_directory, "control_torques.csv")
obj_pose_error_file = os.path.join(current_directory, "obj_pose_error.csv")
obj_pose_real_file = os.path.join(current_directory, "obj_pose_real.csv")
obj_pose_est_file = os.path.join(current_directory, "obj_pose_est.csv")
obj_pose_ftsensor_file = os.path.join(current_directory, "obj_pose_ftsensor.csv")
obj_pose_camera_file = os.path.join(current_directory, "obj_pose_camera.csv")

# Read the CSV files
arm_pose = pd.read_csv(arm_pose_file)
joint_positions = pd.read_csv(joint_positions_file)
control_torques = pd.read_csv(control_torques_file)
obj_pose_error = pd.read_csv(obj_pose_error_file)
obj_pose_real = pd.read_csv(obj_pose_real_file)
obj_pose_est = pd.read_csv(obj_pose_est_file)
obj_pose_ftsensor = pd.read_csv(obj_pose_ftsensor_file)
obj_pose_camera = pd.read_csv(obj_pose_camera_file)

# Convert DataFrame columns to NumPy arrays
arm_pose_data = arm_pose.to_numpy()
joint_positions_data = joint_positions.to_numpy()
control_torques_data = control_torques.to_numpy()
obj_pose_error_data = obj_pose_error.to_numpy()
obj_pose_real_data = obj_pose_real.to_numpy()
obj_pose_est_data = obj_pose_est.to_numpy()
obj_pose_ftsensor_data = obj_pose_ftsensor.to_numpy()
obj_pose_camera_data = obj_pose_camera.to_numpy()

# Figure 1: Arm Data
plt.figure(figsize=(14, 8))
plt.suptitle('Arm Data', fontsize=16)

# Subplot 1: Arm Pose
plt.subplot(2, 2, 1)
plt.plot(arm_pose_data[:, 0], arm_pose_data[:, 1], label='x')
plt.plot(arm_pose_data[:, 0], arm_pose_data[:, 2], label='y')
plt.plot(arm_pose_data[:, 0], arm_pose_data[:, 3], label='z')
plt.title('Arm Pose')
plt.xlabel('Timestamp')
plt.ylabel('Position')
plt.legend()

# Subplot 2: Joint Positions
plt.subplot(2, 2, 2)
for i in range(1, joint_positions_data.shape[1]):
    plt.plot(joint_positions_data[:, 0], joint_positions_data[:, i], label=f'q{i}')
plt.title('Joint Positions')
plt.xlabel('Timestamp')
plt.ylabel('Joint Angle')
plt.legend()

# Subplot 3: Control Torques
plt.subplot(2, 2, 3)
for i in range(1, control_torques_data.shape[1]):
    plt.plot(control_torques_data[:, 0], control_torques_data[:, i], label=f'tau{i}')
plt.title('Control Torques')
plt.xlabel('Timestamp')
plt.ylabel('Torque')
plt.legend()

plt.tight_layout()

# Figure 2: Object Data
plt.figure(figsize=(14, 8))
plt.suptitle('Object Data', fontsize=16)

# Subplot 1: Object Pose Error
plt.subplot(2, 2, 1)
plt.plot(obj_pose_error_data[:, 0], obj_pose_error_data[:, 1], label='X Error')
plt.plot(obj_pose_error_data[:, 0], obj_pose_error_data[:, 2], label='Y Error')
plt.title('Object Pose Error')
plt.xlabel('Timestamp')
plt.ylabel('Error')
plt.legend()

# Subplot 2: Real and Estimated Object Pose
plt.subplot(2, 2, 2)
plt.plot(obj_pose_real_data[:, 0], obj_pose_real_data[:, 1], label='Real X')
plt.plot(obj_pose_real_data[:, 0], obj_pose_real_data[:, 2], label='Real Y')
plt.plot(obj_pose_est_data[:, 0], obj_pose_est_data[:, 1], label='Estimated X')
plt.plot(obj_pose_est_data[:, 0], obj_pose_est_data[:, 2], label='Estimated Y')
plt.title('Real and Estimated Object Pose')
plt.xlabel('Timestamp')
plt.ylabel('Position')
plt.legend()

# Subplot 3: Object Pose by Force/Torque Sensor
plt.subplot(2, 2, 3)
plt.plot(obj_pose_ftsensor_data[:, 0], obj_pose_ftsensor_data[:, 1], label='X_obj')
plt.plot(obj_pose_ftsensor_data[:, 0], obj_pose_ftsensor_data[:, 2], label='Y_obj')
plt.title('Object Pose by Force/Torque Sensor')
plt.xlabel('Timestamp')
plt.ylabel('Position')
plt.legend()

# Subplot 4: Object Pose by Camera Sensor
plt.subplot(2, 2, 4)
plt.plot(obj_pose_camera_data[:, 0], obj_pose_camera_data[:, 1], label='X_obj')
plt.plot(obj_pose_camera_data[:, 0], obj_pose_camera_data[:, 2], label='Y_obj')
plt.title('Object Pose by Camera Sensor')
plt.xlabel('Timestamp')
plt.ylabel('Position')
plt.legend()

plt.tight_layout()
plt.show()

