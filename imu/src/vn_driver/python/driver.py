#!/usr/bin/env python
import rospy
import time 
import serial
import numpy as np
from vn_driver.msg import Vectornav
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
import sys


def isVNYMRinString(inputString):
    if inputString.startswith("$VNYMR"): 
        return True
    else:
        return False

def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(serialPortAddr, baudrate=115200) 
    serialPort.write("$VNWRG,07,40*XX".encode("utf-8"))
    vnymrRead =  str(serialPort.readline().decode('utf-8'))
    serialPort.close() 
    return vnymrRead

def convert_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def rospub():
    pub = rospy.Publisher('\imu', Vectornav, queue_size=100)
    rospy.init_node('vn_driver', anonymous=True)
    #rate = rospy.Rate(10) # 10h
    
    rospy.loginfo("Initialization complete")
    print("Initialized")

    serialPortAddr = sys.argv[1]

    while not rospy.is_shutdown(): 
        vnymrRead = ReadFromSerial(serialPortAddr)
    

        datareceived = isVNYMRinString(vnymrRead)
        if datareceived == True:
             
            try:
                vnymrSplit = vnymrRead.split(",")

                yaw = float(vnymrSplit[1])
                pitch = float(vnymrSplit[2])
                roll = float(vnymrSplit[3])

                magx = float(vnymrSplit[4])
                magy = float(vnymrSplit[5])
                magz = float(vnymrSplit[6])

                accx = float(vnymrSplit[7])
                accy = float(vnymrSplit[8])
                accz = float(vnymrSplit[9])

                angx = float(vnymrSplit[10])
                angy = float(vnymrSplit[11])
                angz = float(vnymrSplit[12].split("*")[0])

            except:
                continue

            (qx, qy, qz, qw) = convert_to_quaternion(yaw, pitch, roll)

            my_msg = Vectornav()
            my_msg.header.frame_id = "IMU_Frame"
            my_msg.header.stamp.secs = int(time.time())
            my_msg.header.stamp.nsecs = int(time.time_ns()) % 10**9
            
            my_msg.imu = Imu()
            
            my_msg.imu.orientation.x = float(qx)
            my_msg.imu.orientation.y = float(qy)
            my_msg.imu.orientation.z = float(qz)
            my_msg.imu.orientation.w = float(qw)

            my_msg.imu.angular_velocity.x = float(angx)
            my_msg.imu.angular_velocity.y = float(angy)
            my_msg.imu.angular_velocity.z = float(angz)

            my_msg.imu.linear_acceleration.x = float(accx)
            my_msg.imu.linear_acceleration.y = float(accy)
            my_msg.imu.linear_acceleration.z = float(accz)
            
            my_msg.mag_field = MagneticField()
            
            my_msg.mag_field.magnetic_field.x = float(magx)
            my_msg.mag_field.magnetic_field.y = float(magy)
            my_msg.mag_field.magnetic_field.z = float(magz)

            my_msg.imu_read = str(vnymrRead)

            rospy.loginfo(my_msg)
            pub.publish(my_msg)

        else:
            print("vnymr is not present.")


if __name__ == '__main__':
    try:
        rospub()
    except rospy.ROSInterruptException:
        pass
