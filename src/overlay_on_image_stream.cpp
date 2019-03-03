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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include <sstream>

using namespace std;
using namespace tf;
using namespace cv;

# define M_PI       3.14159265358979323846  /* pi */


// std::shared_ptr<tf::TransformListener> listener;
// tf::StampedTransform transformSparkCamToWorld;

tf::TransformListener *listener;
tf::StampedTransform transformSparkCamToWorld;
tf::StampedTransform transformPersonToCam;
std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
image_geometry::PinholeCameraModel cam_model_;
char imagePath[100], imageFileName[100];
bool shouldSaveImages=0;


void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    
    
    
    try
    {
        listener->lookupTransform("world", "dji_spark_red_gps_receiver",ros::Time(0), transformSparkCamToWorld);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        //ros::Duration(1.0).sleep();
        //continue;
    }
    
/*    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    } */   

        printf("transformSparkCamToWorld.getOrigin().x() = %f,\t transformSparkCamToWorld.getOrigin().y() = %f,\t transformSparkCamToWorld.getOrigin().z() = %f,\n" ,transformSparkCamToWorld.getOrigin().x(),transformSparkCamToWorld.getOrigin().y(),transformSparkCamToWorld.getOrigin().z());
        printf("transformSparkCamToWorld.roatation.x = %f,\t transformSparkCamToWorld.roatation.y = %f,\t transformSparkCamToWorld.roatation.z = %f \t transformSparkCamToWorld.roatation.w = %f,\n" ,transformSparkCamToWorld.getRotation().x(),transformSparkCamToWorld.getRotation().y(),transformSparkCamToWorld.getRotation().z(),transformSparkCamToWorld.getRotation().w());
         tf::Quaternion q(transformSparkCamToWorld.getRotation().x(),transformSparkCamToWorld.getRotation().y(),transformSparkCamToWorld.getRotation().z(),transformSparkCamToWorld.getRotation().w());
         double roll, pitch, yaw;
         tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        printf("roll= %f,\t pitch = %f,\t yaw = %f,\n" ,roll*(180/M_PI),pitch*(180/M_PI),yaw*(180/M_PI));
        
        
//     tf::Transform transform;
//     transform.setOrigin(tf::Vector3(10, 5, 3));
//     transform.setRotation(tf::Quaternion(0,0,0,1));
//     tf_broadcaster->sendTransform(tf::StampedTransform(transform,msg->header.stamp,"world","Person"));
    
    char obstacleName[10];
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }    
    for (int i=1; i<=3; i++)
    {

        sprintf(obstacleName,"Obstacle%d",i);    
        
        try
        {
            listener->lookupTransform("spark_camera", obstacleName,ros::Time(0), transformPersonToCam);
        }
        catch (tf::TransformException &ex) 
        {
            ROS_ERROR("%s",ex.what());
            //ros::Duration(1.0).sleep();
            //continue;
        }
        
        double X,Y,Z,x_p,y_p,CamX,CamY,CamZ,x_c,y_c,f=0.025,lambda;
        
        X = transformPersonToCam.getOrigin().x(); Y = transformPersonToCam.getOrigin().y(); Z = transformPersonToCam.getOrigin().z();
        
        CamZ = X; CamX = Y, CamY = -Z;
        
        lambda = 1/CamZ;
        
        x_c = lambda*CamX; 
        y_c = lambda*CamY;
        
        // array([[1.83644945e+03, 0.00000000e+00, 9.74958569e+02],
        // [0.00000000e+00, 1.84786663e+03, 4.95976453e+02],
        // [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        
        //         1909.499146 0.000000 970.845520 0.000000
        //         0.000000 1920.237427 503.995288 0.000000
        //         0.000000 0.000000 1.000000 0.000000

        
        //double Fx = 1.83644945e+03, Fy = 1.84786663e+03, Cx = 970.845520, Cy = 503.995288;
    
        double Fx = 1909.499146, Fy = 1920.237427, Cx = 9.74958569e+02, Cy = 4.95976453e+02;
        
        x_p = Fx*x_c + 0*y_c  + Cx;
        y_p = 0*x_c  + Fy*y_c + Cy;
        
        
        cv::Point3d pt_cv(CamX, CamY, CamZ);
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(pt_cv);
        
        
        
    //     _M1 = cvMat(3, 3, CV_64F, M1);
    //     _D1 = cvMat(1, 5, CV_64F, D1);
        
        printf("In Cam Actual World X= %f,  Y= %f,  Z= %f\n" ,X,Y,Z);
        
        printf("In Cam Rotated World X= %f,  Y= %f,  Z= %f,  ........ In image x= %f,\t y = %f\n" ,CamX,CamY,CamZ,x_p,y_p);
        
        printf("In Cam U= %f,  V= %f\n" ,uv.x,uv.y);

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > y_p+40 && cv_ptr->image.cols > x_p+40)
        {
            cv::circle(cv_ptr->image, cv::Point(x_p, y_p), 30, CV_RGB(0,0,0),-1);
            int fontFace = FONT_HERSHEY_DUPLEX;
            double fontScale = 1;
            int thickness = 1;
            cv::putText(cv_ptr->image, obstacleName, cv::Point(x_p+30, y_p), fontFace, fontScale, Scalar::all(255), thickness, 8);
            if (i==1)
                cv::putText(cv_ptr->image, "(0,0)", cv::Point(x_p+30, y_p+30), fontFace, fontScale, Scalar::all(255), thickness, 8);
            if (i==2)
                cv::putText(cv_ptr->image, "(9,-4)", cv::Point(x_p+30, y_p+30), fontFace, fontScale, Scalar::all(255), thickness, 8);
            if (i==3)
                cv::putText(cv_ptr->image, "(9,4)", cv::Point(x_p+30, y_p+30), fontFace, fontScale, Scalar::all(255), thickness, 8);            
            //cv::circle(cv_ptr->image, uv, 20, CV_RGB(255,0,255),2);
        }
    }
    // Update GUI Window
    //v::imshow("view", cv_ptr->image);
    //cv::waitKey(3);
    if(shouldSaveImages)
    {
        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(95);        
        sprintf(imageFileName,"%s/img%05d.jpg",imagePath,msg->header.seq),
        cv::imwrite(imageFileName,cv_ptr->image,compression_params);       
        
    }

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());    
        

}

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
     cam_model_.fromCameraInfo(info_msg);

  
}


void dji_pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    
    
        //printf("x = %f,\t y = %f,\t z = %f,\n" ,msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        //printf("Orient x = %f,\t Orient y = %f,\t Orient z = %f \t Orient w = %f,\n" ,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        //printf("roll= %f,\t pitch = %f,\t yaw = %f,\n" ,roll*(180/M_PI),pitch*(180/M_PI),yaw*(180/M_PI));        
 
  
}

int main(int argc, char **argv)
{

    if ( argc != 3 )
    {
        printf("usage: rosrun rospackage rosnode <PathToSaveImages> <should save images or not 0 or 1>\n");
        return -1;            
    }
    sprintf(imagePath,"%s",argv[1]); 
    shouldSaveImages = atoi(argv[2]);
    
    ros::init(argc, argv, "overlay_on_image_stream");

    ros::NodeHandle n;
    int count = 0;
    
    
    ros::Publisher overlaidImagePublisher;
    
    // Initialize publishers
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/spark_red/image_overlaid", 50);

    // Subscribe to raw images and copter poses converted from GPS poses.
    image_transport::Subscriber raw_image_sub = it.subscribe("/dji_spark_red/image_raw", 50, &image_callback);
    ros::Subscriber image_info_sub = n.subscribe("/dji_spark_red/camera_info", 50, &camera_info_callback);
   
    ros::Subscriber imu_sub = n.subscribe("/dji_spark_red/pose", 50, &dji_pose_callback);
    
    cv::namedWindow("view");
    //cv::startWindowThread();
    
    tf::TransformListener lr(ros::Duration(10));
    listener=&lr;       
    tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();
    
    ros::spin();    
  

}
