import rospy
import utm
import time 
import datetime
import serial
from std_msgs.msg import Header
from gps_driver.msg import Customgps
import sys

def isGPGGAinString(inputString):
    if inputString.startswith("$GPGGA"): #replace 1 == 1 with condition to be checked for inputString
        return True
    else:
        return False

def degMinstoDegDec(LatOrLong):
    deg = (LatOrLong/100)//1 # Replace 0 with a line of code that gets just the degrees from LatOrLong
    mins = round(((LatOrLong/100) % 1 ) * 100, 4) #Replace 0 with a line of code that gets just the minutes from LatOrLong
    degDec = mins/60 #Replace 0 with a line of code that converts minutes to decimal degrees
    #print(LatOrLong)
    #print(deg)
    #print(mins)
    #print(deg+degDec)
    return (deg+degDec)

def LatLongSignConvetion(LatOrLong, LatOrLongDir):
    if LatOrLongDir in ["W","S"]: #Replace the blank string with a value
        LatOrLong = -1 * LatOrLong #some code here that applies negative convention
        #print(LatOrLong)
    #else:

        #print(LatOrLong)
    return LatOrLong

def convertToUTM(LatitudeSigned, LongitudeSigned):
    UTMVals = utm.from_latlon(LatitudeSigned, LongitudeSigned)
    UTMEasting = UTMVals[0] #Again, replace these with values from UTMVals
    UTMNorthing = UTMVals[1]
    UTMZone = UTMVals[2]
    UTMLetter = UTMVals[3]
    #print(UTMVals)
    return [UTMEasting, UTMNorthing, UTMZone, UTMLetter]

def UTCtoUTCEpoch(UTC):
    #print(UTC)
    UTChours = int(UTC/10000)
    UTCmin = int((UTC%10000)/100)
    UTCsec = ((UTC%10000)%100)
    UTCinSecs = (UTChours * 3600) + (UTCmin*60) + UTCsec  
    TimeSinceEpochBOD = (datetime.datetime.now(datetime.timezone.utc).date()- datetime.datetime(1970,1,1).date()).total_seconds()
    CurrentTime = TimeSinceEpochBOD + UTCinSecs
    CurrentTimeSec = int(CurrentTime) 
    CurrentTimeNsec = int((CurrentTime - CurrentTimeSec) * 1e9 ) 
    return [CurrentTimeSec, CurrentTimeNsec]

def ReadFromSerial(serialPortAddr):
    serialPort = serial.Serial(serialPortAddr, baudrate=4800) 
    gpggaRead =  serialPort.readline() 
    serialPort.close() 
    return str(gpggaRead)[2:]

def rospub():
    pub = rospy.Publisher('gps', Customgps, queue_size=100)
    rospy.init_node('gps_driver', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    rospy.loginfo("Initialization complete")
    print("Initialized")

    serialPortAddr = sys.argv[1]

    while not rospy.is_shutdown(): 
        gpggaRead = ReadFromSerial(serialPortAddr)

        datareceived = isGPGGAinString(gpggaRead)
        if datareceived == True:
            gpggaSplit = gpggaRead.split(",") #Put code here that will split gpggaRead into its components. This should only take one line.
            #print(gpggaSplit)a

            UTC = gpggaSplit[1]
            Latitude = gpggaSplit[2]
            LatitudeDir = gpggaSplit[3]
            Longitude = gpggaSplit[4]
            LongitudeDir = gpggaSplit[5]
            HDOP = gpggaSplit[8]
            altitude = gpggaSplit[9]
            #print(UTC)
            try:

                UTC = float(gpggaSplit[1])
                Latitude = float(gpggaSplit[2])
                LatitudeDir = str(gpggaSplit[3])
                Longitude = float(gpggaSplit[4])
                LongitudeDir = str(gpggaSplit[5])
                HDOP = float(gpggaSplit[8])
                altitude = float(gpggaSplit[9])
            except:
                continue
            #print(UTC)

            Latitude= degMinstoDegDec(Latitude)
            Longitude= degMinstoDegDec(Longitude)



            LatitudeSigned = LatLongSignConvetion(Latitude, LatitudeDir)
            LongitudeSigned = LatLongSignConvetion(Longitude, LongitudeDir)
            print(LatitudeSigned, LongitudeSigned)

            UTMdata = convertToUTM(LatitudeSigned, LongitudeSigned)

            CurrentTime = UTCtoUTCEpoch(UTC)

            print("sending out the message")
            gps_msg = Customgps()
            gps_msg.header.frame_id = "GPS1_Frame"
            gps_msg.header.stamp.secs = CurrentTime[0]
            gps_msg.header.stamp.nsecs = CurrentTime[1]
            gps_msg.latitude = LatitudeSigned
            gps_msg.longitude = LongitudeSigned
            gps_msg.altitude = altitude
            gps_msg.utm_easting = UTMdata[0]
            gps_msg.utm_northing = UTMdata[1]
            gps_msg.zone = UTMdata[2]
            gps_msg.letter = UTMdata[3]
            gps_msg.hdop = HDOP
            gps_msg.gpgga_read = gpggaRead

            rospy.loginfo(gps_msg)
            pub.publish(gps_msg)

        else:
            print("GPGGA is not present.")


if __name__ == '__main__':
    try:
        rospub()
    except rospy.ROSInterruptException:
        pass