#!/usr/bin/env python

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

import load_novatel_data, convert_novatel_to_pose

novatel_path = '/home/tariqul/repos/cadc_data/cadcd_data/2018_03_06/0001/novatel/data/';

novatel = load_novatel_data.load_novatel_data(novatel_path);
poses_rfu = convert_novatel_to_pose.convert_novatel_to_pose(novatel);
poses_flu = convert_novatel_to_pose.convert_novatel_to_pose(novatel, 'FLU');

debug_rfu = poses_rfu[0][0:3, 0:3]
debug_flu = poses_flu[0][0:3, 0:3]

def set_axes_equal(ax):
    '''Make the axes of a 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's ax.set_aspect('equal')
    limitation in 3D.'''
    
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in this method
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
def is_orthogonal(R):
    should_be_identity = np.dot(R, R.T)
    identity = np.eye(3)
    return np.allclose(should_be_identity, identity, atol=1e-8)


if not is_orthogonal(debug_rfu):
    print("RFU rotation matrix is not orthogonal!")
if not is_orthogonal(debug_flu):
    print("FLU rotation matrix is not orthogonal!")






mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_title('Vehicle path')
ax.set_xlabel('East (m)')
ax.set_ylabel('North (m)')
ax.set_zlabel('Up (m)')

length = 1
A  = np.matrix([[0, 0, 0, 1],
               [length, 0, 0, 1],
               [0, 0, 0, 1],
               [0, length, 0, 1],
               [0, 0, 0, 1],
               [0, 0, length, 1]]).transpose()

# A_flu = np.matrix([[0, 0, 0, 1],   # Origin (0, 0, 0)
#                [0, -length, 0, 1],  # Y-axis: 1 unit along the Y (Left)
#                [0, 0, 0, 1],   # Origin (0, 0, 0)
#                [length, 0, 0, 1],  # X-axis: 1 unit along the X (Front)
#                [0, 0, 0, 1],   # Origin (0, 0, 0)
#                [0, 0, length, 1]]).transpose()  # Z-axis: 1 unit along the Z (Up)

num_frames = 50
skip = 100
# for pose_rfu in poses_rfu:
for i, pose_rfu in enumerate(poses_rfu):
  if i>skip:
    if i%5 == 0: 
      B = np.matmul(pose_rfu, A)
      ax.plot([B[0,0], B[0,1]], [B[1,0], B[1,1]],[B[2,0],B[2,1]], 'r-', label='RFU X' if i == 0 else ""); # x: red
      ax.plot([B[0,2], B[0,3]], [B[1,2], B[1,3]],[B[2,2],B[2,3]], 'g-', label='RFU Y' if i == 0 else ""); # y: green
      ax.plot([B[0,4], B[0,5]], [B[1,4], B[1,5]],[B[2,4],B[2,5]], 'b-', label='RFU Z' if i == 0 else ""); # z: blue
    if (i-skip)/5 >=num_frames:
     break
for i, pose_flu in enumerate(poses_flu):
  if i>skip:
    if i % 5 == 0:  # Optional: Plot every 5th pose for clarity
        B = np.matmul(pose_flu, A)
        ax.plot([B[0,0], B[0,1]], [B[1,0], B[1,1]], [B[2,0], B[2,1]], 'r--', label='FLU X' if i == 0 else "")
        ax.plot([B[0,2], B[0,3]], [B[1,2], B[1,3]], [B[2,2], B[2,3]], 'g--', label='FLU Y' if i == 0 else "")
        ax.plot([B[0,4], B[0,5]], [B[1,4], B[1,5]], [B[2,4], B[2,5]], 'b--', label='FLU Z' if i == 0 else "")
# Equal axis doesn't seem to work so set an arbitrary limit to the z axis
    if (i-skip)/5 >=num_frames:
     break
ax.set_zlim3d(-10,10)
set_axes_equal(ax)

plt.show()
