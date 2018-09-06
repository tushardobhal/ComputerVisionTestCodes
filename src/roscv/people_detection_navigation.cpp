#include <iostream>
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
#include <opencv/cv.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;


static const char WR[] = "COLOR Image";
static const char WD[] = "DEPTH Image";
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

  //! Clean up the action client
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
class Gripper{
private:
  GripperClient* gripper_client_;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);

    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
  }

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = 0.08;
    open.command.max_effort = -1.0;  // Do not limit effort (negative)

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.0;
    squeeze.command.max_effort = 50.0;  // Close gently

    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};
class ImageConverter:public Gripper,public RobotArm
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher image_pub_;
  
public:
  

  cv::Mat depth_frame;
  cv::Mat img;
  float x,y,z,cx_d,fx_d;
  HOGDescriptor hog;
  vector<Rect> found, found_filtered;

//Constructor for class ImageConverter. 
//Publishes and Subscribes to Color and Depth Image Topics and Initializes Variables 
ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    depth_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthInfoCb, this);
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    cv::namedWindow(WR,CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WD, CV_WINDOW_AUTOSIZE);
    fx_d = 1.0 / 594.21434211923247;
    cx_d = 339.30780975300314;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WR);
    cv::destroyWindow(WD);
  }   

//Gets the Distance from the Kinect of the points passed by the OnMouse function and Stores them into variables
//Then gives the command to the Robot to Move

void getdepth(int  c1,int c2)
{
    x =  depth_frame.at<float>(c2,c1);
    z = 1.25;
    y = ((cx_d - c1) * x * fx_d);
    x = x - 0.9;
	
    cout<<"x = "<<x<<"\ty = "<<y<<"\tz = "<<z<<endl;
    MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;

  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
    RobotArm arm;
  Gripper gripper;
  //untuck the arms
  float pre_five_r []= {-0.40004820592173013, 0.99991199109465412, 3.9552329405445619e-05,-2.0500712507403946,0.0,-0.09984264424337963, -3.7293354441692017e-05 };
  float pre_five_l []= {0.39988239268701614, 0.99999658601775865,-3.9552329405223574e-05,-2.0499264800290109,0.0,-0.09999548327039598, -9.323334379196524e-06 };
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_l,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_r,1.0,true),true);

    sleep(2.0);
  //handing over objects
   float post_five_r []={-0.0258077351723,0.0346839184728,0.0702749662794, -0.144743918222,0.00815643926003,-0.0795931465837,-0.0400788872061};
   float post_five_l []={0.39988239268701614, 0.99999658601775865,-3.9552329405223574e-05,-2.0499264800290109,0.0,-0.09999548327039598, -9.323334379196524e-06 };
  arm.startTrajectory(arm.arm_trajectoryPoint(post_five_l,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(post_five_r,1.0,true),true);

    char o,c;
    cout<<"press o to open the gripper"<<endl;
    cin>>o;
    gripper.open();
    cout<<"press c to close the gripper"<<endl;
    cin>>c;
    gripper.close();

}

//Listens to the Mouse Event and Sends the Points to the function getdepth()

static void onMouse(int event, int a, int b, int flags, void *this_)
{
   
  if(event == CV_EVENT_LBUTTONDOWN)

      static_cast<ImageConverter*>(this_)->getdepth(a, b);
}

//Converts Ros Color Images to OpenCV Mat Images
//Detects People and Draws a Rectangle around them
//Calls the function onMouse() if a Mouse Event occurs 

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
   
     
        hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
        size_t i, j;
        for (i=0; i<found.size(); i++) 
        {
            Rect r = found[i];
            for (j=0; j<found.size(); j++) 
                if (j!=i && (r & found[j]) == r)
                    break;
            if (j== found.size())
                found_filtered.push_back(r);
        }
 
        for (i=0; i<found_filtered.size(); i++) 
        {
            Rect r = found_filtered[i];
            r.x += cvRound(r.width*0.1);
		    r.width = cvRound(r.width*0.8);
		    r.y += cvRound(r.height*0.07);
		    r.height = cvRound(r.height*0.8);
		    rectangle(img, r.tl(), r.br(), Scalar(0,255,0), 3);        
        }
      
      found.clear();
      found_filtered.clear();
 
    
    setMouseCallback(WR, onMouse, this);   


    cv::imshow(WR, img);
     if(cv::waitKey(3)==27)
              exit(0);
    
    image_pub_.publish(cv_ptr_rgb->toImageMsg());
  }

//Converts ROS Depth Image to OpenCV Mat Image used to Get the Depth of a Point

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
    if(cv::waitKey(3)==27)
              exit(0);
    
    depth_pub_.publish(cv_ptr_depth->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
//  ros::spin();
/*    RobotArm arm;
  Gripper gripper;
  //untuck the arms
  float pre_five_r []= {-0.40004820592173013, 0.99991199109465412, 3.9552329405445619e-05,-2.0500712507403946,0.0,-0.09984264424337963, -3.7293354441692017e-05 };
  float pre_five_l []= {0.39988239268701614, 0.99999658601775865,-3.9552329405223574e-05,-2.0499264800290109,0.0,-0.09999548327039598, -9.323334379196524e-06 };
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_l,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(pre_five_r,1.0,true),true);

    sleep(2.0);
  //handing over objects
   float post_five_r []={-0.0258077351723,0.0346839184728,0.0702749662794, -0.144743918222,0.00815643926003,-0.0795931465837,-0.0400788872061};
   float post_five_l []={0.39988239268701614, 0.99999658601775865,-3.9552329405223574e-05,-2.0499264800290109,0.0,-0.09999548327039598, -9.323334379196524e-06 };
  arm.startTrajectory(arm.arm_trajectoryPoint(post_five_l,1.0,false),false);
  arm.startTrajectory(arm.arm_trajectoryPoint(post_five_r,1.0,true),true);

    char o,c;
    cout<<"press o to open the gripper"<<endl;
    cin>>o;
    gripper.open();
    cout<<"press c to close the gripper"<<endl;
    cin>>c;
    gripper.close(); */
    ros::spin();
  return 0;

  
}


