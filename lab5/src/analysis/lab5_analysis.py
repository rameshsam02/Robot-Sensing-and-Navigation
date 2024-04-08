import bagpy
from bagpy import bagreader
import pandas as pd 
import matplotlib.pyplot as plt
import scipy.integrate as integrate
from scipy.signal import butter, sosfilt, freqz
from scipy.optimize import least_squares 
import numpy as np
import math

imu_bag = bagreader('/home/vboxuser/catkin_ws/src/data/data_going_in_circles_imu.bag') #circle imu
imu_data_ten = imu_bag.message_by_topic('/imu')
imu_data_ten_csv = pd.read_csv(imu_data_ten)

mag_x_imu = imu_data_ten_csv['magfield.magnetic_field.x']
mag_y_imu = imu_data_ten_csv['magfield.magnetic_field.y']
mag_z_imu = imu_data_ten_csv['magfield.magnetic_field.z']

ten_min_data_bag = bagreader('/home/vboxuser/catkin_ws/src/data/data_driving_imu.bag') 
moving = bagreader('/home/vboxuser/catkin_ws/src/data/data_driving_gps.bag') 

imu_data_ten_r = ten_min_data_bag.message_by_topic('/imu')
gpsmsg1 = moving.message_by_topic('/gps')

imu_data_csv = pd.read_csv(imu_data_ten_r)
moving = pd.read_csv(gpsmsg1)

mag_x = imu_data_csv['magfield.magnetic_field.x']
mag_y = imu_data_csv['magfield.magnetic_field.y']
mag_z = imu_data_csv['magfield.magnetic_field.z']

linear_acceleration = imu_data_csv[['imu.linear_acceleration.x','imu.linear_acceleration.y', 'imu.linear_acceleration.z']]
magnetic_field = imu_data_csv[['magfield.magnetic_field.x','magfield.magnetic_field.y', 'magfield.magnetic_field.z']]
angular_velocity = imu_data_csv[['imu.angular_velocity.x','imu.angular_velocity.y', 'imu.angular_velocity.z']]
time = imu_data_csv['Time']

def distortion_model(X_meas, dist_params):
    x = dist_params[0] * (X_meas[0] - dist_params[4]) + dist_params[1]*(X_meas[1] - dist_params[5])
    y = dist_params[2] * (X_meas[0] - dist_params[4]) + dist_params[3]*(X_meas[1] - dist_params[5])
    X = np.array([x,y])
    return X
    
def residual(p, X_mag, X_meas):
    return (X_mag - distortion_model(X_meas, p)).flatten()

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    r = 6371000
    return c * r

#plotting trajectory using dead reckoning from imu data
def calculate_trajectory(velocity_data, yaw_data, time_data):
    x = 0
    y = 0
    x_positions = []
    y_positions = []

    for i in range(len(velocity_data)):
        displacement_x = (velocity_data[i] * np.cos(np.radians(yaw_data[i])))/50
        displacement_y = (velocity_data[i] * np.sin(np.radians(yaw_data[i])))/50
        x += displacement_x
        y += displacement_y
        x_positions.append(-x)
        y_positions.append(y)

    return x_positions, y_positions


#circle
field_strength = 20509e-5 
angle = np.linspace(-np.pi*5, np.pi*5, len(mag_x_imu))
x_mag = field_strength * np.sin(angle) 
y_mag = field_strength * np.cos(angle) 
X_mag = np.array([x_mag, y_mag])

x_meas = mag_x_imu
y_meas = mag_y_imu
X_meas = np.array([x_meas, y_meas])

p0 = [0,0,0,0,0,0]
lsq_min = least_squares(residual, p0, args=(X_mag, X_meas))
X_model = distortion_model(X_meas, lsq_min.x) 

#Plotting ellipse and lsq
plt.figure(figsize=(5,5))
plt.scatter(X_model[0],X_model[1], label="Calibrated data", s=1)
plt.scatter(X_meas[0],X_meas[1], label="Measured data", s=1)
plt.axis('equal')
plt.legend()
plt.xlabel('X component of magnetic field (Gauss)')
plt.ylabel('Y component of magnetic field (Gauss)')
plt.title('Magnetometer Data Before and After Calibration vs. Time')

plt.tight_layout()
plt.show()

#ride
x_meas_r = mag_x
y_meas_r = mag_y
X_meas_r = np.array([x_meas_r, y_meas_r])
X_model_r = distortion_model(X_meas_r, lsq_min.x) 

#Plotting ellipse and lsq
plt.figure(figsize=(5,5))
plt.scatter(X_model_r[0],X_model_r[1], label="Calibrated data", s = 1)
plt.scatter(X_meas_r[0],X_meas_r[1], label="Measured data", s = 1)
plt.axis('equal')
plt.legend()
plt.xlabel('X component of magnetic field (Gauss)')
plt.ylabel('Y component of magnetic field (Gauss)')
plt.title('Magnetometer Data Before and After Calibration vs. Time')

plt.tight_layout()
plt.show()

#Magnetometer Heading Before and After Calibration vs. Time
t = np.array(time)
plt.figure(figsize=(10,6))

cal_mag_heading = np.degrees(np.arctan2(mag_x,mag_y))
X_meas_r = np.array([mag_x, mag_y])
X_model_r = distortion_model(X_meas_r, lsq_min.x)
cal_mag_heading_c = np.degrees(np.arctan2(X_model_r[0],X_model_r[1]))

plt.plot(t, np.array(cal_mag_heading), color='blue', label="Uncalibrated data")
plt.plot(t, np.array(cal_mag_heading_c), color='orange', label="Calibrated data")
plt.xlabel('Time (s)')
plt.ylabel('Magnetometer Heading (degrees)')
plt.title('Magnetometer Heading Before and After Calibration vs. Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig("rideMagnetometerHeading")
plt.show()

#Gyro yaw
rate_z = angular_velocity['imu.angular_velocity.z']
rotation_z = np.degrees(integrate.cumulative_trapezoid(rate_z, t, initial = 0))   

plt.plot(t, rotation_z, color='red', label='Z Axis', markersize=10)
plt.xlabel('Time (s)')
plt.ylabel('Gyro Yaw (deg)')
plt.title('Gyro - Z Axis')
plt.legend()
plt.grid(True)

#Filters
plt.figure(figsize=(10, 8))

sampl_freq = 40
cutoff_freq_low = 0.01
cutoff_freq_high = 0.02

t = np.array(time)

data1 = (cal_mag_heading_c)
data2 = (rotation_z)

sos, y1 = butter_filter(data1, cutoff_freq_low, sampl_freq, "lowpass", 2)
sos, y2 = butter_filter(data2, cutoff_freq_high, sampl_freq, "highpass", 2)

#complementary
a = 0.95
comp = a*y1 + (1-a)*y2

#quarternions to yaw
x = np.array(imu_data_csv['imu.orientation.x'])
y = np.array(imu_data_csv['imu.orientation.y'])
z = np.array(imu_data_csv['imu.orientation.z'])
w = np.array(imu_data_csv['imu.orientation.w'])

t3 = +2.0 * (w * z + x * y)
t4 = +1.0 - 2.0 * (y * y + z * z)
raw_yaw = np.degrees(np.arctan2(t3, t4))

plt.subplot(2, 2, 1)
plt.plot(t, data1, "b-", label="data")
plt.plot(t, y1, "g-", linewidth=2, label="filtered data")
plt.xlabel("Time [sec]")
plt.ylabel('Yaw(degrees)')
plt.title("Low pass filter data")
plt.grid()
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(t, data2, "b-", label="data")
plt.plot(t, y2, "g-", linewidth=2, label="filtered data")
plt.xlabel("Time [sec]")
plt.ylabel('Yaw(degrees)')
plt.title("High pass filter data")
plt.grid()
plt.legend()

plt.subplot(2, 2, 3)
plt.plot(t, comp, "b-", label="data")
plt.xlabel("Time [sec]")
plt.ylabel('Yaw (degrees)')
plt.title("Complementary filter data")
plt.grid()
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(t, raw_yaw, "g-", linewidth=2, label="filtered data")
plt.xlabel("Time [sec]")
plt.ylabel('Yaw (degrees)')
plt.title("IMU Heading Estimate")
plt.grid()
plt.legend()
plt.subplots_adjust(hspace=0.40, wspace=0.30)
plt.show()

#Velocity from IMU
#before adjustments
import math

t = np.array(time)
plt.figure(figsize=(10, 6))

accx = linear_acceleration['imu.linear_acceleration.x']
accy = linear_acceleration['imu.linear_acceleration.y']

b_vel_x = 3.6 * (integrate.cumtrapz(accx, t, initial = 0))
b_vel_y = 3.6 * (integrate.cumtrapz(accy, t, initial = 0))

b_vel = np.sqrt((b_vel_x**2) + (b_vel_y**2))
angle = np.arctan2(b_vel_y,b_vel_x)
sign_1 = np.cos(angle)
b_vel_with_sign = (np.sign(sign_1))*b_vel

plt.subplot(2, 1, 1)
plt.plot(t, b_vel_with_sign, color='blue', label='Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/h)')
plt.title('Velocity from IMU (Before Adjustment)')
plt.legend()
plt.grid(True)

#after adjustment
sampl_freq_vel = 40
cutoff_freq_vel = 0.004

sos, v1 = butter_filter(accx, cutoff_freq_vel, sampl_freq_vel, "highpass", 2)
sos, v2 = butter_filter(accy, cutoff_freq_vel, sampl_freq_vel, "highpass", 2)

vel_x = 3.6 * (integrate.cumtrapz(v1, t, initial = 0))
vel_y = 3.6 * (integrate.cumtrapz(v2, t, initial = 0))

vel = np.sqrt((vel_x**2) + (vel_y**2))

plt.subplot(2, 1, 2)
plt.plot(t, vel, color='blue', label='Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/h)')
plt.title('Velocity from IMU (After Adjustment)')
plt.legend()
plt.grid(True)
plt.subplots_adjust(hspace=0.5)
plt.show()

#gps velocity
t = np.array(time)

gps_lat = moving['latitude']
gps_long = moving['longitude']
gps_time = moving['Time']

velocities = []
distances = []

for i in range(1, len(gps_lat)):
    lat1 = gps_lat[i-1]
    lon1 = gps_long[i-1]
    time1 = gps_time[i-1]

    lat2 = gps_lat[i]
    lon2 = gps_long[i]
    time2 = gps_time[i]

    distance = haversine(lat1, lon1, lat2, lon2) 
    distances.append(distance)

total_dist = np.cumsum(distances)

velocity=[]
for i in range(0,len(distances)):
    v = distances[i]/(gps_time[i+1] - gps_time[i])
    velocity.append(v)

plt.plot(np.array(gps_time[0:1193]), np.array(velocity[0:1193]), label='Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity from GPS Sensor Data')
plt.legend()
plt.show()

#plotting trajectory from gps data
plt.figure(figsize=(14, 6))

velocity_data = vel/3.6
yaw_data = raw_yaw + 42
time_data = t

x_positions, y_positions = calculate_trajectory(velocity_data, yaw_data, time_data)

plt.plot(moving['utm_easting'].to_numpy() - moving['utm_easting'][0], moving['utm_northing'].to_numpy() - moving['utm_northing'][0], label='GPS Trajectory')
plt.plot(x_positions, y_positions, label='IMU Trajectory')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Estimated Trajectory using Dead Reckoning')
plt.legend()
plt.grid(True)
plt.show()

#vel from gps and imu
plt.figure(figsize=(8, 4))
plt.plot(t, b_vel_with_sign/3.6, color='blue', label='Velocity IMU')
plt.plot(np.array(gps_time[0:1193]), np.array(velocity[0:1193]), label='Velocity GPS')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity from GPS and IMU')
plt.legend()
plt.grid(True)
plt.show()

#acc
vx = integrate.cumtrapz(v1, t, initial = 0)
plt.figure(figsize=(8, 4))
plt.plot(t, linear_acceleration['imu.linear_acceleration.y'].to_numpy(), color='blue', label='y double dot')
plt.plot(t, angular_velocity['imu.angular_velocity.z'].to_numpy() * vx, label='w*xdot', color = 'orange')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s2)')
plt.title('Acceleration in Y double dot vs. w*x dot')
plt.legend()
plt.grid(True)
plt.show()

