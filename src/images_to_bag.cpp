#include <istream>
#include <fstream>
#include <iostream>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "csv.h"
#include "geodesy/utm.h"
#include "geodesy/wgs84.h"
#include <rosbag/bag.h>
#include <time.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <sstream>

using namespace std;
using namespace tf;
using namespace cv;

# define M_PI       3.14159265358979323846  /* pi */

int main(int argc, char **argv)
{

    if ( argc != 4 )
    {
    printf("usage: rosrun rospackage rosnode <Full Path where the Video is and inside that folder there should be a folder named with images with split videos are> <Output Bagname> <date time info file> \n");
    return -1;            
    }
    
    
    ros::init(argc, argv, "images_to_bag");

    ros::NodeHandle n;
    int count = 0;
    
    // Initialize the bag and start writing in it
    rosbag::Bag bag;
    char bagFileName[100],logfileName[100];
    sprintf(bagFileName,"%s/%s.bag",argv[1],argv[2]);
    sprintf(logfileName,"%s/image_id_timestamp.log",argv[1]);
    bag.open(bagFileName, rosbag::bagmode::Write);    
    
    // Initialize time related structs and variables
    //Set up time
    int firstSecStamp;
    time_t rawtime;
    struct tm * timeinfo;
    time_t unix_time,unix_time_base_for_image,unix_time_base_for_data;
    int year,month,day,hour,min,sec,nsec;
    int offsetMilSec;
    long int rosTimeSec = 0;  int rosTimeNsec = 0;   
    
  
    // File reading the image id and timestamps
    FILE *imageFileLogHandler; long int imageID=0, pts_image=0, image_sec=0, imagensec=0;
    float image_time=0.0;
    imageFileLogHandler = fopen(logfileName,"r");
    //fscanf(imageFileLogHandler,"n:%d pts: %d pts_time:%d.%d\n",&imageID,&pts_image,&image_sec,&imagensec);
    
    //Load the base time for the images
    // e.g. 2016_02_26_21_17_51.info
    sscanf(argv[3],"%d_%d_%d_%d_%d_%d.info",&year,&month,&day,&hour,&min,&sec);
    /* get current timeinfo: */
    //time ( &rawtime ); //or: rawtime = time(0);
    /* convert to struct: */
    //timeinfo = localtime ( &rawtime ); 
    /* now modify the timeinfo to the given date: */
    timeinfo->tm_year   = year - 1900;
    timeinfo->tm_mon    = month - 1;    //months since January - [0,11]
    timeinfo->tm_mday   = day;          //day of the month - [1,31] 
    timeinfo->tm_hour   = hour;         //hours since midnight - [0,23]
    timeinfo->tm_min    = min;          //minutes after the hour - [0,59]
    timeinfo->tm_sec    = sec;          //seconds after the minute - [0,59]

    /* call mktime: create unix time stamp from timeinfo struct */
    unix_time_base_for_image = timegm ( timeinfo );  
    ros::Time imageMsgTime;
    
    cv::Mat image;
    std_msgs::Header header;
    char mystring [100];
    char filename[11];    
    
    while(!feof(imageFileLogHandler))
    {
        fgets (mystring , 100 , imageFileLogHandler);
        printf("%s\n",mystring);
        
        if(imageID < 9999)
            sscanf(mystring,"n:%4d pts:%7ld pts_time:%f\n",&imageID,&pts_image,&image_time);
        else
            sscanf(mystring,"n:%5d pts:%7ld pts_time:%f\n",&imageID,&pts_image,&image_time);
        
        imageMsgTime = ros::Time(unix_time_base_for_image+(int)image_time, 1000000000*(image_time-(int)image_time));
        
        //Create image message
        
        //Create image filename, e.g., fr00001.jpg

        sprintf(filename,"%s/images/fr%05d.png",argv[1],imageID);

        
        printf("image file is %s\n",filename);
        
        header.stamp = imageMsgTime;
        header.frame_id = "spark_cam_image";
        header.seq = imageID;
        image = cv::imread(filename, IMREAD_COLOR);
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();        
  
        printf("We are here \n");       
        printf("processed file fr%d.jpg AT pts_image = %d time = %d.%f\n",imageID,pts_image,unix_time_base_for_image+(int)image_time,1000000000*(image_time-(int)image_time));
         
        bag.write ("/spark_red/image_raw", ros::Time::now(), img_msg);        
    }


    bag.close();


    return 0;
}
