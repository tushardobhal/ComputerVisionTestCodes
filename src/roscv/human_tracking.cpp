#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

namespace enc = sensor_msgs::image_encodings; 
class DepthInfo
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Subscriber depth_sub_;
image_transport::Publisher depth_pub_;
/*
image_transport shouLd always be used to subscribe to and publish images. ___,
It provides transparent support for transporting images in Low-bandwidth compressed formats.
//www. ros. org/wiki/imag e_t ranspo rt
*/
public:
cv::Mat rgb_frame; 
cv::Mat depth_frame; 
ros::Publisher cmd_vel_pub_;
geometry_msgs::Twist base_cmd;
int prev_info;
DepthInfo(): it_(nh_)
{
cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1); //publishing
depth_pub_ = it_.advertise("out", 1); 
depth_sub_ = it_.subscribe("camera/depth/image", 1, &DepthInfo::depthInfoCb, this); //subscribes
image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &DepthInfo::imageCb, this); //subscribesi
prev_info=4;
}
void imageCb(const sensor_msgs::ImageConstPtr & msg) 
{ 
cv_bridge::CvImagePtr cv_ptr_rgb;
//cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actuall
try
{
cv_ptr_rgb = cv_bridge::toCvCopy(msg, enc::BGR8); 
}
catch (cv_bridge::Exception& e)
{
ROS_ERROR("cv_bridge exception: ks", e.what());
return;
}

rgb_frame = cv_ptr_rgb->image; 
cv::circle(rgb_frame, cv::Point(20, 240), 5, cv::Scalar(255, 0, 0), 2); 
cv::circle(rgb_frame, cv::Point(320, 240), 5, cv::Scalar(255, 0, 0), 2);
cv::circle(rgb_frame, cv::Point(620, 240), 5, cv::Scalar(255, 0, 0), 2);
//circ1e(Mat& irng, Point center, int radius, const Sca'Lar& color, int 1ineType)
cv::imshow( " rgb_image" , rgb_frame) ;
cv::waitKey(3 );
}



void depthInfoCb(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImagePtr cv_ptr_depth;
try {cv_ptr_depth = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);}
catch (cv_bridge::Exception&  e)
{
ROS_ERROR("cv_bridge exception: %s", e.what());

return;
}
depth_frame = cv_ptr_depth->image;
cv::imshow("depth_image", depth_frame);
int info=get_depthInfo();
cout<<"info"<<info<<endl;
 

if(info==1) cv::circle(depth_frame, cv::Point(20, 240), 5, cv::Scalar(0, 0, 0), 2);
else if(info==2) cv::circle(depth_frame, cv::Point(320, 246), 5, cv::Scalar(0, 6, 6), 2); 
else if(info==3) cv::circle(depth_frame, cv::Point(620, 246), 5, cv::Scalar(0, 0, 6), 2);
else 
{

cv::circle(depth_frame, cv::Point(20, 240), 5, cv::Scalar(255, 255, 255), 2);
cv::circle(depth_frame, cv::Point(320, 246), 5, cv::Scalar(255, 255, 255), 2);
cv::circle(depth_frame, cv::Point(620, 246), 5, cv::Scalar(255, 255, 255), 2);
}


cv::imshow("depth_1mage", depth_frame); 
depth_pub_.publish(cv_ptr_depth->toImageMsg( ) );


base_cmd.angular.x=0; 
base_cmd.angular.y=0; 
base_cmd.angular.z=0; 
base_cmd.linear.x=0; 
base_cmd.linear.y=0;
base_cmd.linear.z=0; 

//adding lines for continous navigation




if(info==1)//turn right (yaw) and drive forward at the same time }
{
string right = "right";
string right_command = "echo \"" + right  + "\" | festival --tts";
system(right_command.c_str());

base_cmd.angular.z = -0.75; 
base_cmd.linear.x = 1.0;
} 
else if(info==2) //move forward
{
string forward = "forward";
string forward_command = "echo \"" + forward  + "\" | festival --tts";
system(forward_command.c_str());
base_cmd.linear.x = 1.0;
}

else if(info==3)//turn left (yaw) and drive forward at the same time else
{
string left = "left";
string left_command = "echo \"" + left  + "\" | festival --tts";
system(left_command.c_str());
base_cmd.angular.z = 0.75; 
base_cmd.linear.x = 1.0; 
}

else //stop
{
base_cmd.angular.x=0;
base_cmd.angular.y=0;
base_cmd.angular.z=0; 
base_cmd.linear. x=0;
base_cmd.linear.y=0; 
base_cmd.linear.z=0;
} 
cmd_vel_pub_.publish(base_cmd); 

//prev_info =info;
//cout<<"prev_info"<<prev_info<<endl;
}

int get_depthInfo()
{
int posX_d = 20; // cots
int posY_d = 240; // rows
int posX_c = 320; // cots
int posY_c = 240; // rows
int posX_e = 620; // cols
int posY_e = 240; // rows
double depthInfo_e,depthInfo_d,depthInfo_c;
depthInfo_e = depth_frame.at<float>(posY_e, posX_e);
depthInfo_d = depth_frame.at<float>(posY_d, posX_d);
depthInfo_c = depth_frame.at<float>(posY_c, posX_c);
if(depthInfo_d>0.600 && depthInfo_d<1.000) 
{ 

return 3;
}
else if(depthInfo_c>0.600 && depthInfo_c<1.000) 
{

return 2;
}
else if(depthInfo_e>0.600 && depthInfo_e<1.000) 
{

return 1;
}
else
{return 0;
} 

}
};

int main(int argc, char** argv)
{
 ros::init(argc,argv,"depth_info");
 DepthInfo ic;
 ros::spin();
 return 0;
}

