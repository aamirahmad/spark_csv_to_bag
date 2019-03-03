# DJI Spark flight data to ROS bags

The package can be used to convert DJI spark flight data (GPS pose and gimbal orientation) and on-board video to rosbags.

## Important practical information before Using this package

1. This package works on the following assumptions and preparatory steps
    1. During a flight, flight data is recorded by DJI spark on the host mobile phone connected to the remote controller.
    2. This flight data is available as a *.txt file (e.g., DJIFlightRecord_2016-01-13_[20-42-10].txt) in the folder ../dji.go.v4/FlightRecord/ inside the DJI folder on the phone. This text file has the flight records encrypted, hence follow the next step.
    3. Go to the service website https://app.airdata.com/ and upload the above text file and download a csv version of the flight data (e.g., 2016-02-26_21-16-27_Standard.csv). To do so you need an account on this webservice. It is free to make and no c.card is required. You can convert up to 100 flights. Then delete old flights and convert new ones. So basically you can use it indefinitely if you keep just download a local copy of csv and delete it on the server after conversion.
    4. We will be using the above downloaded csv file to convert them to rosbags.

2. Not all flights may have a video recorded on board. However, most of them do have it. Hence, to convert the video into rosbag we will need to do additional steps like extracting frames out of the video. These steps are not mandatory and can be skipped. If skipped, be careful with the arguments you pass to the executable of the nodes explained later in this readme. 

3. In case a video is recorded and being converted to rosbag, pay special attention to the start time of the video. This is also explained later in appropriate place.

## Description of files and folders

1. src -- Contains source codes
    1. flightdata_to_bag.cpp -- This is the main source code that converts the flight data csv and video file into a rosbag.
    2. images_to_bag.cpp -- This is a test source code to verify the conversion of only video file to rosbag. 
    3. 

... To Be Completed

