
# Convert LL to UTM
import utm
import numpy as np
import math
from scipy.spatial.transform import Rotation 

# Converts GPS data to poses in the ENU frame
def convert_novatel_to_pose(novatel, body_frame = 'RFU'):
  poses = [];
  FIRST_RUN = True;
  origin = [];
  def rotx(t):
    """Rotation about the x-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[1, 0, 0], [0, c, s], [0, -s, c]])
  def roty(t):
      """Rotation about the y-axis."""
      c = np.cos(t)
      s = np.sin(t)
      return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]]) 
  def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
  
  for gps_msg in novatel:
    
    # utm_data[0] = East (m), utm_data[1] = North (m)
    utm_data = utm.from_latlon(float(gps_msg[0]), float(gps_msg[1]));
    # Ellipsoidal height = MSL (orthometric) + undulation
    ellipsoidal_height = float(gps_msg[2]) + float(gps_msg[3]);

    roll = np.deg2rad(float(gps_msg[7]));
    pitch = np.deg2rad(float(gps_msg[8]));
    # Azimuth = north at 0 degrees, east at 90 degrees, south at 180 degrees and west at 270 degrees
    azimuth = float(gps_msg[9]);
    # yaw = north at 0 deg, 90 at west and 180 at south, east at 270 deg
    yaw = np.deg2rad(-1.0 * azimuth); 


    c_phi = math.cos(roll);
    s_phi = math.sin(roll);
    c_theta = math.cos(pitch);
    s_theta = math.sin(pitch);
    c_psi = math.cos(yaw);
    s_psi = math.sin(yaw);

    if FIRST_RUN:
      origin = [utm_data[0], utm_data[1], ellipsoidal_height];
      FIRST_RUN = False;

    # This is the T_locallevel_body transform where ENU is the local level frame
    # and the imu is the body frame
    # https://www.novatel.com/assets/Documents/Bulletins/apn037.pdf
    # we go from body fame to local frame and do matrix multiplciaiton of the axis vector of body frame in lcoal frame
    # which gives us the axes vector of body fame in body frame thus plottinmg the trajectory
    if body_frame == 'RFU':
      # poses.append(np.array([
      #   [c_psi * c_phi - s_psi * s_theta * s_phi, -s_psi * c_theta, c_psi * s_phi + s_psi * s_theta * c_phi, utm_data[0] - origin[0]],
      #   [s_psi * c_phi + c_psi * s_theta * s_phi, c_psi * c_theta, s_psi * s_phi - c_psi * s_theta * c_phi, utm_data[1] - origin[1]],
      #   [-c_theta * s_phi, s_theta, c_theta * c_phi, ellipsoidal_height - origin[2]],
      #   [0.0, 0.0, 0.0, 1.0]]))
      R =rotz(-yaw).dot(rotx(-pitch).dot(roty(-roll)))   #transform from body frame to local frame
      # R = (roty(roll).dot(rotx(pitch).dot(rotz(yaw)))) #transform from local frame to body frame
      T = np.zeros((4,4))
      T[:3, :3]= R
      T[0, 3] = utm_data[0] - origin[0]
      T[1, 3] = utm_data[1] - origin[1]  
      T[2, 3] = ellipsoidal_height - origin[2]
      T[3, 3] = 1
      poses.append(T)
    if body_frame == 'FLU':

      R_z_90_T = np.array([[0, 1, 0],
                         [-1, 0, 0],
                         [0, 0, 1]])
      #R = (roty(roll).dot(rotx(pitch).dot(rotz(yaw)))).dot(R_z_90_T)      #transform from local frame to body frame   
      R = (R_z_90_T).T.dot((roty(roll).dot(rotx(pitch).dot(rotz(yaw)))).T) #transform from body frame to local frame  
      T = np.zeros((4,4))
      T[:3, :3]= R
      T[0, 3] = utm_data[0] - origin[0]
      T[1, 3] = utm_data[1] - origin[1]  
      T[2, 3] = ellipsoidal_height - origin[2]
      T[3, 3] = 1
      poses.append(T)


  return poses;
