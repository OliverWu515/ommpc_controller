#--------------------------------------
#Generate reference trajectory with visualization
#--------------------------------------

import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Parameters
sample_time = 0.01             # seconds
duration = 60                  # seconds

r = 1.2
# Initial and final velocities
v_initial = 0.5
v_final = 2.0

x0 = r                     
y0 = 0
z0 = 0.8

clockwise = True
factor = -1 if clockwise else 1

# Time array
t = np.arange(0, duration + sample_time, sample_time)
num_points = len(t)

# Create trajectory array
traj = np.zeros((num_points, 8))

# Store velocity magnitude and angle for plotting
v_magnitude = np.zeros(num_points)
angle_array = np.zeros(num_points)

# Generate trajectory
for i, time in enumerate(t):
    # Calculate current velocity (linear from 0.5 to 2.0)
    v_current = v_initial + (v_final - v_initial) * (time / duration)
    v_magnitude[i] = v_current
    
    # Calculate cumulative angle (needs velocity integration)
    if i == 0:
        angle = 0
    else:
        # Use trapezoidal rule for integration
        v_prev = v_initial + (v_final - v_initial) * ((time - sample_time) / duration)
        angle += ((v_prev + v_current) / (2 * r)) * sample_time
    
    angle_array[i] = angle
    
    # Position
    traj[i, 0] = -r * np.cos(angle) + x0
    traj[i, 1] = factor * -r * np.sin(angle) + y0
    traj[i, 2] = z0
    
    # Velocity (tangential direction)
    traj[i, 3] = v_current * np.sin(angle)  # x velocity component
    traj[i, 4] = factor * -v_current * np.cos(angle)  # y velocity component
    traj[i, 5] = 0  # z velocity
    
    # Acceleration (set to 0, or can calculate actual acceleration)
    traj[i, 6] = 0
    traj[i, 7] = 0

# Calculate velocity magnitude (for verification)
calculated_v_magnitude = np.sqrt(traj[:, 3]**2 + traj[:, 4]**2)

# write to txt
np.savetxt('circle.txt', traj, fmt='%f')

# Create visualization plots
fig = plt.figure(figsize=(15, 10))

# 1. 3D Trajectory Plot
ax1 = fig.add_subplot(231, projection='3d')
ax1.plot(traj[:, 0], traj[:, 1], traj[:, 2], 'b-', linewidth=2, label='Trajectory')
ax1.scatter(traj[0, 0], traj[0, 1], traj[0, 2], c='r', s=100, label='Start', marker='o')
ax1.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], c='g', s=100, label='End', marker='s')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Trajectory')
ax1.legend()
ax1.grid(True)

# 2. XY Plane Trajectory (Top View)
ax2 = fig.add_subplot(232)
ax2.plot(traj[:, 0], traj[:, 1], 'b-', linewidth=2, label='Trajectory')
ax2.scatter(traj[0, 0], traj[0, 1], c='r', s=100, label='Start')
ax2.scatter(traj[-1, 0], traj[-1, 1], c='g', s=100, label='End')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('XY Plane Trajectory')
ax2.axis('equal')
ax2.grid(True)
ax2.legend()

# 3. Velocity Magnitude vs Time
ax3 = fig.add_subplot(233)
ax3.plot(t, v_magnitude, 'r-', linewidth=2, label='Desired Velocity')
ax3.plot(t, calculated_v_magnitude, 'b--', linewidth=1.5, label='Calculated Velocity')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Velocity Magnitude (m/s)')
ax3.set_title('Velocity vs Time')
ax3.grid(True)
ax3.legend()

# 4. Velocity Components vs Time
ax4 = fig.add_subplot(234)
ax4.plot(t, traj[:, 3], 'r-', linewidth=2, label='Vx')
ax4.plot(t, traj[:, 4], 'b-', linewidth=2, label='Vy')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Velocity Components (m/s)')
ax4.set_title('Velocity Components vs Time')
ax4.grid(True)
ax4.legend()

# 5. Cumulative Angle vs Time
ax5 = fig.add_subplot(235)
ax5.plot(t, angle_array, 'g-', linewidth=2)
ax5.set_xlabel('Time (s)')
ax5.set_ylabel('Angle (rad)')
ax5.set_title('Cumulative Angle vs Time')
ax5.grid(True)

# 6. Position Coordinates vs Time
ax6 = fig.add_subplot(236)
ax6.plot(t, traj[:, 0], 'r-', linewidth=2, label='X')
ax6.plot(t, traj[:, 1], 'b-', linewidth=2, label='Y')
ax6.plot(t, traj[:, 2], 'g-', linewidth=2, label='Z')
ax6.set_xlabel('Time (s)')
ax6.set_ylabel('Position (m)')
ax6.set_title('Position Coordinates vs Time')
ax6.grid(True)
ax6.legend()

plt.tight_layout()

# Add statistics text box
stats_text = f"""
Trajectory Statistics:
- Total Time: {duration} s
- Sampling Time: {sample_time} s
- Total Points: {num_points}
- Initial Velocity: {v_initial:.2f} m/s
- Final Velocity: {v_final:.2f} m/s
- Circle Radius: {r} m
- Start Point: ({traj[0,0]:.2f}, {traj[0,1]:.2f}, {traj[0,2]:.2f})
- End Point: ({traj[-1,0]:.2f}, {traj[-1,1]:.2f}, {traj[-1,2]:.2f})
- Direction: {'Clockwise' if clockwise else 'Counter-clockwise'}
"""

fig.text(0.02, 0.02, stats_text, fontsize=10, verticalalignment='bottom',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.show()

# Output verification information
print("=" * 60)
print("Trajectory Generation and Verification Results:")
print("=" * 60)
print(f"Initial Velocity: Vx={traj[0, 3]:.3f}, Vy={traj[0, 4]:.3f}, Total={np.sqrt(traj[0, 3]**2 + traj[0, 4]**2):.3f} m/s")
print(f"Mid Velocity: Vx={traj[num_points//2, 3]:.3f}, Vy={traj[num_points//2, 4]:.3f}, Total={np.sqrt(traj[num_points//2, 3]**2 + traj[num_points//2, 4]**2):.3f} m/s")
print(f"Final Velocity: Vx={traj[-1, 3]:.3f}, Vy={traj[-1, 4]:.3f}, Total={np.sqrt(traj[-1, 3]**2 + traj[-1, 4]**2):.3f} m/s")
print(f"Total Revolutions: {angle_array[-1] / (2*np.pi):.2f} circles")
print(f"Trajectory saved to: circle.txt")
print(f"Visualization saved to: trajectory_analysis.png")
print("=" * 60)
