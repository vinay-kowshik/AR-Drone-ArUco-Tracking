// Basic C++ Includes
#include <iostream>
#include <math.h>
// Includes for OpenCV Libraries
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
// Includes for ArUco Libraries
#include <opencv2/aruco.hpp>
// Includes for ROS Libraries
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace cv;
using namespace aruco;

static const std::string OPENCV_WINDOW = "Image Window";
Mat InImage(480, 640, CV_8UC3);	// To store converted ROS Video Stream Frames in OpenCV compatible Matrix Format
bool firstImageReceived;
cv_bridge::CvImagePtr cv_ptr;	// Pointer created under cv_bridge package to temporarily store ROS converted image
Mat camMatrix, distCoeffs;	// Matrix Variables initialized to store Calibration parameters - Camera matrix & Distortion Coefficients

class ImageConverter
{
ros::NodeHandle nh_;		// ROS Node Initialization to handle various ROS Topics like cmd_vel, takeoff, land etc.
image_transport::ImageTransport it_;	// Initializing variable "it_" under image_transport package
image_transport::Subscriber image_sub_;	// Initializing variable "image_sub_" to subscribe Drone front camera video stream
ros::Publisher pub;	// Initializing variable "pub" to publish various topics to Drone through ROS Node (like cmd_vel, takoff etc.)
tf::TransformListener listener;
tf::StampedTransform transform;	// Initializing variable "transform" under 'tf' package used to transform coordinate frames
geometry_msgs::Twist twist;	// Initializing variable "twist" under geometry_msgs used to control linear & rotation motion of drone
geometry_msgs::Twist twist_old;	// Initializing variable "twist_old" to set linear & angular velocities to zero when loss of marker

vector<int> markerIds;		// To store Marker IDs
vector<Vec3d> rvecs, tvecs;	// To store rotational and translational vectors
vector <vector<Point2f> > markerCorners, rejectedCandidates; // For storing Marker Corners data and Rejected Candidates
aruco::DetectorParameters parameters; 
Ptr <aruco::Dictionary> markerDictionary;  // To store 4x4 Marker Dictionary 

public: 
ImageConverter() : it_(nh_)
{
 string CALIB_FILE_PATH = "/home/han/Desktop/frontcam_calib.yml"; // Calling the calibration file to access calibration parameters; Note that the path mentioned changes corresponding to the file location located in the computer
setCalibFilePath(CALIB_FILE_PATH);
image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, &ImageConverter::imageCb, this);  // Subscribing the Front Camera Video Stream
namedWindow(OPENCV_WINDOW);	// Opens a New Window named "Image Window"
markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(0)); // Defining the Marker Dictionary. Here '0' indicates the index assigned to access 4x4 ArUco Marker Dictionary. Below listed the index no.s for respective Dictionary

/* Following are the list of indices for different Marker Dictionaries
1. DICT_4x4_50 = 0 (read as "4x4 aruco markers of quantity 50 no.s (i.e., From Marker ID 0 to 49)")
2. DICT_4x4_100 = 1
3. DICT_4x4_250 = 2
4. DICT_4x4_1000 = 3
5. DICT_5x5_50 = 4
6. DICT_5x5_100 = 5
7. DICT_5x5_250 = 6
8. DICT_5x5_1000 = 7
9. DICT_6x6_50 = 8
10. DICT_6x6_100 = 9
11. DICT_6x6_250 = 10
12. DICT_6x6_1000 = 11
13. DICT_7x7_50 = 12
14. DICT_7x7_100 = 13
15. DICT_7x7_250 = 14
16. DICT_7x7_1000 = 15
17. DICT_ARUCO_ORIGINAL = 16 */

// Setting Initial States of Drone to Zero
twist.linear.x = 0.0;
twist.linear.y = 0.0;
twist.linear.z = 0.0;
twist.angular.x = 0.0;
twist.angular.y = 0.0;
twist.angular.z = 0.0;

tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
try
{
  listener.lookupTransform("/ardrone_base_frontcam", "marker", ros::Time(0), transform);
  // Transformation Logic to Transform Camera frame of reference to Drone Frame of Reference
  twist.linear.x = (transform.getOrigin().z()-1)*0.1;	// Forward - Backward Motion
  twist.linear.y = -transform.getOrigin().x()*0.2;	// Left - Right Motion
  twist.linear.z = -transform.getOrigin().y()*2.0;	// Up - down motion
}
catch(tf::TransformException &ex)
{
  ROS_ERROR("%s", ex.what());
  return;
}

// Incase of Lost of Marker control from Camera field of view
if(twist.linear.x == twist_old.linear.x && twist.linear.y==twist_old.linear.y && twist.linear.z==twist_old.linear.z)
{
  twist.linear.x=0.0;
  twist.linear.y=0.0;
  twist.linear.z=0.0;
  twist.angular.x=0.0;
  twist.angular.y=0.0;
  twist.angular.z=0.0;
}
else
{
  twist_old = twist;
}
}

~ImageConverter()
{
destroyWindow(OPENCV_WINDOW);
}
  
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
   ROS_INFO("Frontcam : image received");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);	// Image Conversion using cv_bridge
   
    }
    catch (cv_bridge::Exception& e)
{
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
}
    InImage = cv_ptr->image;
    // imwrite("Sample.jpg",InImage);
// cout<<"Input :"<<InImage.at<Vec3b>(Point(320,240))<<endl;

aruco::detectMarkers(InImage, markerDictionary, markerCorners, markerIds);
//cout<<"Aruco : "<<InImage.at<Vec3b>(Point(320,240))<<endl;

cout <<"\nmarker Size is:\t"<<markerIds.size()<<endl;

aruco::drawDetectedMarkers(InImage, markerCorners, markerIds);
 if(markerIds.size() > 0){
        cout <<"\nmarker ID is:\t"<<markerIds[0]<<endl;
	aruco::estimatePoseSingleMarkers(markerCorners, 0.17f, camMatrix, distCoeffs, rvecs, tvecs);
 for(int i = 0; i < markerIds.size(); i++)
 {
   aruco::drawAxis(InImage, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1f);
   // cout <<"\nRotational Vectors:\t"<< rvecs[i] << endl;
   // cout <<"\nTranslational Vectors:\t"<< tvecs[i]  << endl;
 }

  
 // Take off and Landing - with Marker Detection
 if (markerIds[0]==8)
 {
  pub = nh_.advertise <std_msgs::Empty>("/ardrone/takeoff",1, true);	// Publishes the takeoff message to drone
  pub.publish(std_msgs::Empty()); //Launches the drone
  //ros::spinOnce();
 }

 if(markerIds[0] == 8)
 {
   pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);		// Publishes the velocity message to drone
   pub.publish(twist);
 }

 if (markerIds[0]==11)
 {
  pub = nh_.advertise <std_msgs::Empty>("/ardrone/land", 1, true);	// Publishes the landing message to drone
  pub.publish(std_msgs::Empty());
  //ros::spinOnce();
 }
}
  
/* Getting the position of marker
 x = tvecs[0];
 y = tvecs[1];
 z = tvecs[2]; 

/* To get the Euler Angles
Rodrigues(rvecs, rot_mat); // Rodrigues() func converts 3x1 vector to 3x3 matrix
 cout<<"Rotational Matrix:"<<rot_mat<<endl;*/

 imshow(OPENCV_WINDOW, InImage);	// Displays the Video stream of Drone Front cam
 waitKey(3);
}

// To read the cameral calibration file with .yml extension
void setCalibFilePath(string str)
{
  FileStorage fs2(str, FileStorage::READ);

  fs2["cameraMatrix"]>>camMatrix;
  fs2["distortion_coeffs"]>>distCoeffs;
  //cout<<camMatrix<<distCoeffs<<endl;
}
};

// Main Function to call class "ImageConverter"
int main(int argc, char** argv)
{
 ros::init(argc, argv, "aruco_detection");
 ImageConverter ic;
 ros::spin();
 return 0;
}
 

    
