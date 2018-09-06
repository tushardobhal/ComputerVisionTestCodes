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
#include <math.h>
//#include <hmm/CvHMM.h>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WR[] = "COLOR Image";
static const char WD[] = "DEPTH Image";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher image_pub_;
  
public:
  

cv::Mat img, depth_frame, im1;
float x[100], y[100], ang[100], xdiff[100], ydiff[100];
int z;
Mat imp, imf, imm, imp1, hip, hif, him, hip1;


ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    depth_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthInfoCb, this);
     z = 0;
   

      
      cv::namedWindow(WR,CV_WINDOW_AUTOSIZE);
     
      cv::namedWindow(WD, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WR);
    cv::destroyWindow(WD);
  }


void display(float d[])
{
   for(int i=0;i<z;i++)
   {  
     cout<<d[i]<<" ";
   }  
   cout<<endl;
}


int vectorise(float f)
{
   int a;
   if((f>=315 && f<360)||(f>=0 && f<45))
        a = 0;
   if(f>=45 && f<135)
        a = 1;
   if(f>=135 && f<225)
        a = 2;
   if(f>=225 && f<315) 
        a = 3;
   return a;
}
 

void contour()
{
   Rect rect1, rect2;
   Mat im = im1.clone();
   Moments m = moments(im, true);
   Point cen(m.m10/m.m00, m.m01/m.m00);
   x[z] = cen.x; 
   y[z] = cen.y;
   if(z!=0)
     {
        ang[z] = atan2(y[z] - y[z-1], x[z] - x[z-1])*(180/CV_PI);
        if(ang[z] < 0) ang[z] = 360 + ang[z];
        ang[z] = vectorise(ang[z]);
        //xdiff[z-1] = x[z] - x[z-1];
        //ydiff[z-1] = y[z] - y[z-1];
        cout<<"ang"<<endl;
        display(ang);
      }
   ++z;
}


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
     cvtColor(img, img, CV_BGR2YUV);
     imshow(WR, img);
     if(cv::waitKey(30)==27)
              exit(0);
    
     image_pub_.publish(cv_ptr_rgb->toImageMsg());
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
     
    if(!im1.data)
       im1.create(depth_frame.size(), CV_8UC1);
  
   im1 = Scalar::all(0);

    float min = 10.0;

    for(int i = 0;i < 640; i++)
    {
       for(int j = 0;j < 480; j++)
       {  
           float dist = depth_frame.at<float>(j,i);
           if(dist < min)
               min = dist;
      }
   }


   for(int i = 0; i < 640; i++)
   {
      for(int j = 0; j < 480; j++)
      {
           float dist = depth_frame.at<float>(j,i);
           if((dist > min) && (dist < (min + 0.1)) && (min < 1.5))
           { 
                 im1.at<uchar>(j,i) = 255;
           } 
      }    
   } 
  
    erode(im1,im1,Mat());
    erode(im1,im1,Mat());
    dilate(im1,im1,Mat()); 
 
  
    if(min < 1.5)
    {
      contour();
    }   

    cv::imshow(WD, depth_frame);
    cv::imshow("HAND", im1);
    
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


