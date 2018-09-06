#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"
#include <opencv/cv.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>TrajClient;
typedef actionlib::SimpleActionClient< pr2_controllers_msgs::Pr2GripperCommandAction > GripperClient;




static const char WR[] = "COLOR Image";
static const char WR1[] = "MASK Image";
static const char WR2[] = "FOREGROUND Image";
static const char WD[] = "DEPTH Image";
//adding class for tuckarms and untucking 
class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_r_;
  TrajClient* traj_client_l_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm()
  {
    // tell the action client that we want to spin a thread by default
    traj_client_r_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    traj_client_l_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_l_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
    // wait for action server to come up
    while(!traj_client_r_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }
~RobotArm()
  {
    delete traj_client_r_;
    delete traj_client_l_;
  }
void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,bool right_arm)
  {
    //Start the trjaectory immediately
    goal.trajectory.header.stamp = ros::Time::now();

    if(right_arm)
      traj_client_r_->sendGoal(goal);
    else
      traj_client_l_->sendGoal(goal);
  }
pr2_controllers_msgs::JointTrajectoryGoal arm_trajectoryPoint(float* angles, float duration, bool right_arm)
  {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    //starts at 17
    if(right_arm)
    {
      goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    }
    else
    {
      goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
      goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
      goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
      goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
      goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
      goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
      goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
    }

// We will have N waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] =  angles[0];
    goal.trajectory.points[ind].positions[1] =  angles[1];
    goal.trajectory.points[ind].positions[2] =  angles[2];
    goal.trajectory.points[ind].positions[3] =  angles[3];
    goal.trajectory.points[ind].positions[4] =  angles[4];
    goal.trajectory.points[ind].positions[5] =  angles[5];
    goal.trajectory.points[ind].positions[6] =  angles[6];
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // set time we want this trajectory to be reached at
    goal.trajectory.points[ind].time_from_start = ros::Duration(duration);
   return goal;
  }

};
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher image_pub_;
  
public:
  

cv::Mat depth_frame;
BackgroundSubtractorMOG2 bg_model;
Mat img,fgimg,fgmask,out;
vector<std::vector<cv::Point> > contours;
vector<std::vector<cv::Point> > cont;
double area, maxR, maxG, maxY, maxT;
Point2f cen;
float rad;
//string color;
char c;

ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    depth_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
     depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthInfoCb, this);
      
      maxT = 1000;
      bg_model.nmixtures = 3;
      bg_model.bShadowDetection = false;
      
      cv::namedWindow(WR,CV_WINDOW_AUTOSIZE);
      cv::namedWindow(WR1, CV_WINDOW_AUTOSIZE);
      cv::namedWindow(WD, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WR);
    cv::destroyWindow(WD);
  }

double color(Scalar sc1, Scalar sc2)
{
    if( fgimg.empty() )
       fgimg.create(img.size(), img.type());

        //update the model
        bg_model(img, fgmask, -1);
        
        cv::erode(fgmask,fgmask,cv::Mat());
        
        cv::dilate(fgmask,fgmask,cv::Mat());
        fgimg = Scalar::all(0);
        img.copyTo(fgimg, fgmask);
        
        cvtColor(fgimg,fgimg,CV_BGR2HSV);
        inRange(fgimg, sc1, sc2,out);
        cv::erode(out,out,cv::Mat());
        
        cv::dilate(out,out,cv::Mat());
       double max = 0;
       int x =0;
       findContours(out,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
       for(int i=0;i<contours.size();i++)
       {
            area = contourArea(contours[i]);
            if(area>max)
                {
                  max = area;
                  //cont[0] = contours[i];
                  x=i;
                 }
        }
        
        /*minEnclosingCircle(contours[x], cen, rad);    
 
        if(getdepth(cen.x, cen.y))
        {
            drawContours(img,contours,x,cv::Scalar(0,0,255),2);
            return max;
        }
        
        else
             return 0.0;*/
        drawContours(img,contours,x,cv::Scalar(0,0,255),2);
        return max;
        
}


bool getdepth(int x, int y)
{
    if(depth_frame.at<float>(y,x) < 3)
        return true;
    else
        return false; 
}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
  { //string color;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
     img = cv_ptr_rgb->image;
   
     maxR = color(Scalar(160,120,0), Scalar(180,256,256));
     maxG = color(Scalar(38,120,0), Scalar(75,256,256));
     maxY = color(Scalar(22,120,0), Scalar(35,256,256));
     if((maxR > maxG) && (maxR > maxY) && (maxR > maxT))
           {
             c = 'r';
           }
     if((maxG > maxR) && (maxG > maxY) && (maxG > maxT))
           {
               c = 'g';
           }
     if((maxY > maxR) && (maxY > maxG) && (maxY > maxT))
           {
               c = 'y';
           }
     cvtColor(fgimg, fgimg, CV_HSV2BGR);
     imshow(WR, img);
     imshow(WR1,out); 
     imshow(WR2,fgimg); 
     
     if(cv::waitKey(30)==27)
              exit(0);
    
     image_pub_.publish(cv_ptr_rgb->toImageMsg());
//adding lines for navigation
 if(c=='r')
{  RobotArm arm;

 cout<<"color is red .. stop command is recievd"<<endl;
//untucking the arms wen seeing red color
float pre_five_r []= {-0.40004820592173013, 0.99991199109465412, 3.9552329405445619e-05,-2.0500712507403946,0.0,-0.09984264424337963, -3.7293354441692017e-05 };
  float pre_five_l []= {0.39988239268701614, 0.99999658601775865,-3.9552329405223574e-05,-2.0499264800290109,0.0,-0.09999548327039598, -9.323334379196524e-06 };
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_l,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_r,1.0,true),true);
}
if(c=='y')
{RobotArm arm;
cout<<"getting ready for navigation"<<endl;


string ready ="ready";
string ready_command = "echo \"" + ready  + "\" | festival --tts";
system(ready_command.c_str());







//system("rosrun pr2_tuckarm tuck_arms.py b");
float pre_five_rt []= {  -0.0235692565037,1.10726294851,-1.55668510207,-2.12245660643,-1.41748501864,-1.84170775746,
  0.214436820497
 };
  float pre_five_lt []= {  0.0596336350539,1.2485364701,1.78903924548,-1.68336703881,-1.73437136095,-0.0974786595846,
  -0.0864522242323,
};
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_lt,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_rt,1.0,true),true);


}
if(c=='g')
{
cout<<"I am navigating "<<endl;
MoveBaseClient ac("move_base", true);

while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.0;
 // goal.target_pose.pose.position.y = y;

  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

}  
}

void depthInfoCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr_depth;
    try
    {
      cv_ptr_depth = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    depth_frame = cv_ptr_depth->image;
    
    
    cv::imshow(WD, depth_frame);
    if(cv::waitKey(30)==27)
              exit(0);
    
    depth_pub_.publish(cv_ptr_depth->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


