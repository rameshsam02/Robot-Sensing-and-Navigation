import bagpy
from bagpy import bagreader
import pandas as pd 
import matplotlib.pyplot as plt
import numpy

stationary_occluded = bagreader('/home/vboxuser/2024-02-02-13-16-54.bag') #stationary, occluded
stationary_open = bagreader('/home/vboxuser/2024-02-02-14-08-45.bag') #stationary, open
moving = bagreader('/home/vboxuser/2024-02-02-14-48-59.bag') #moving

gpsmsg1 = stationary_occluded.message_by_topic('/gps')
gpsmsg2 = stationary_open.message_by_topic('/gps')
gpsmsg3 = moving.message_by_topic('/gps')

st_oc = pd.read_csv(gpsmsg1)
st_op = pd.read_csv(gpsmsg2)
moving = pd.read_csv(gpsmsg3)


#subtracting the offset
st_oc['utm_easting_minus_offset'] = st_oc['utm_easting'] - st_oc['utm_easting'][0]
st_oc['utm_northing_minus_offset'] = st_oc['utm_northing'] - st_oc['utm_northing'][0]
st_op['utm_easting_minus_offset'] = st_op['utm_easting'] - st_op['utm_easting'][0]
st_op['utm_northing_minus_offset'] = st_op['utm_northing'] - st_op['utm_northing'][0]
moving['utm_easting_minus_offset'] = moving['utm_easting'] - moving['utm_easting'][0]
moving['utm_northing_minus_offset'] = moving['utm_northing'] - moving['utm_northing'][0]


#finding the centroid
st_oc_cen_e = st_oc["utm_easting_minus_offset"].describe()['mean']
st_oc_cen_n = st_oc["utm_northing_minus_offset"].describe()['mean']
st_oc_std_e = st_oc["utm_easting_minus_offset"].describe()['std']
st_oc_std_n = st_oc["utm_northing_minus_offset"].describe()['std']

st_op_cen_e = st_op["utm_easting_minus_offset"].describe()['mean']
st_op_cen_n = st_op["utm_northing_minus_offset"].describe()['mean']
st_op_std_e = st_op["utm_easting_minus_offset"].describe()['std']
st_op_std_n = st_op["utm_northing_minus_offset"].describe()['std']

moving_cen_e = moving["utm_easting_minus_offset"].describe()['mean']
moving_cen_n = moving["utm_northing_minus_offset"].describe()['mean']
moving_std_e = moving["utm_easting_minus_offset"].describe()['std']
moving_std_n = moving["utm_northing_minus_offset"].describe()['std']


#subtracting the centroid
st_oc['utm_easting_minus_centroid'] = st_oc['utm_easting_minus_offset'] - st_oc_cen_e
st_oc['utm_northing_minus_centroid'] = st_oc['utm_northing_minus_offset'] - st_oc_cen_n

st_op['utm_easting_minus_centroid'] = st_op['utm_easting_minus_offset'] - st_op_cen_e
st_op['utm_northing_minus_centroid'] = st_op['utm_northing_minus_offset'] - st_op_cen_n

moving['utm_easting_minus_centroid'] = moving['utm_easting_minus_offset'] - moving_cen_e
moving['utm_northing_minus_centroid'] = moving['utm_northing_minus_offset'] - moving_cen_n


#stationary open and occluded utm northing vs utm easting
plt.figure(figsize =(10,10))
plt.scatter(st_oc['utm_easting_minus_centroid'], st_oc['utm_northing_minus_centroid'], color = 'b', label='Occluded') 
plt.scatter(st_op['utm_easting_minus_centroid'], st_op['utm_northing_minus_centroid'], color = 'g', label='Open') 
plt.scatter(st_oc_cen_e, st_oc_cen_n, color = 'r', label = 'Centroid in Occluded')
plt.scatter(st_op_cen_e, st_op_cen_n, color = 'orange', label = 'Centroid in Open')
plt.xlabel('Difference in UTM Easting (in meters)') 
plt.ylabel('Difference in UTM Northing (in meters)') 
plt.title('Stationary - Open & Occluded \n UTM Northing vs UTM Easting', fontsize = 15) 
plt.legend()
plt.text(-14.5,-5.5,f"Open\nEasting_Centroid={round(st_op_cen_e,4)}\nNorthing_Centroid={round(st_op_cen_n,4)}\nStd_Dev_Easting={round(st_op_std_e,4)}\nStd_Dev_Northing={round(st_op_std_n,4)}\n\nOcculded\nEasting_Centroid={round(st_oc_cen_e,4)}\nNorthing_Centroid={round(st_oc_cen_n,4)}\nStd_Dev_Easting={round(st_oc_std_e,4)}\nStd_Dev_Northing={round(st_oc_std_n,4)}")
plt.savefig("Stationary_EastVsNorth")
plt.show()

#stationary altitude vs time, open and occluded
plt.figure(figsize =(6,6))
plt.plot(st_oc['Time'].to_numpy(), st_oc['altitude'].to_numpy(), color = 'b', label='Occluded') 
plt.plot(st_op['Time'].to_numpy(), st_op['altitude'].to_numpy(), color = 'g', label='Open')
plt.xlabel('Time (in seconds)') 
plt.ylabel('Altitude (in meters)') 
plt.title('Stationary - Open & Occluded \n Altitude vs Time', fontsize = 15) 
plt.legend()
plt.savefig("Altitude_Stationary")
plt.show()

#histogram for position in occluded - calcuated the euclidean distance from point to centroid
plt.figure(figsize =(6,6))
st_oc['dist_to_cen'] = numpy.sqrt(st_oc['utm_easting_minus_centroid']**2 + st_oc['utm_northing_minus_centroid']**2)
plt.hist(st_oc['dist_to_cen'], label = 'Euclidean Distance')
plt.xlabel('Position') 
plt.ylabel('Number of Rows in that Position') 
plt.title('Histogram for Position in Occluded', fontsize = 15) 
plt.legend()
plt.savefig("Occluded_Histogram")
plt.show()

#histogram for position in open - calcuated the euclidean distance from point to centroid
plt.figure(figsize =(6,6))
st_op['dist_to_cen'] = numpy.sqrt(st_op['utm_easting_minus_centroid']**2 + st_op['utm_northing_minus_centroid']**2)
plt.hist(st_op['dist_to_cen'], label = 'Euclidean Distance')
plt.xlabel('Position') 
plt.ylabel('Number of Rows in that Position') 
plt.title('Histogram for Position in Open', fontsize = 15) 
plt.legend()
plt.savefig("Open_Histogram")
plt.show()

#moving utm northing vs utm easting
plt.figure(figsize =(6,6))
plt.scatter(moving['utm_easting_minus_centroid'], moving['utm_northing_minus_centroid'], color = 'orange', label='Moving')  
a,b = numpy.polyfit(moving['utm_easting_minus_centroid'], moving['utm_northing_minus_centroid'], 1)
plt.plot(numpy.array(moving['utm_easting_minus_centroid']), (a*numpy.array(moving['utm_easting_minus_centroid']))+b, label='Line of best fit')
plt.xlabel('UTM Easting (in meters)') 
plt.ylabel('UTM Northing (in meters)') 
plt.title('Moving \n UTM Northing vs UTM Easting', fontsize = 15) 
plt.legend()
plt.text(5,-50,f"Moving\nEasting_Centroid={round(moving_cen_e,4)}\nNorthing_Centroid={round(moving_cen_n,4)}\nStd_Dev_Easting={round(moving_std_e,4)}\nStd_Dev_Northing={round(moving_std_n,4)}")
plt.savefig("Moving_EastingVsNorthing")
plt.show()

#moving altitude vs time
plt.figure(figsize =(6,6))
plt.plot(moving['Time'].to_numpy(), moving['altitude'].to_numpy(), color = 'b', label='Moving') 
plt.xlabel('Time (in seconds)') 
plt.ylabel('Altitude (in meters)') 
plt.title('Moving \n Altitude vs Time', fontsize = 15) 
plt.legend()
plt.savefig("Moving_Alt_Time")
plt.show()

