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
#include <hmm/CvHMM.h>

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
  
CvHMM hmm;
Mat img, depth_frame, im1;
vector<double> x, y;
vector<int> ang;
int q,z,w;
char t[20];
Mat TRANS1, EMIS1, INIT1;
Mat TRANS2, EMIS2, INIT2;
Mat TRANS3, EMIS3, INIT3;
Mat TRANS4, EMIS4, INIT4;

ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    depth_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthInfoCb, this);
    z = 0;
    q = 0;
    w = 0;
    namedWindow(WR,CV_WINDOW_AUTOSIZE);
    namedWindow(WD, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WR);
    cv::destroyWindow(WD);
  }


void train(Mat &train_seq, Mat &TRANS, Mat &EMIS, Mat &INIT, int st, int obs)
{
   TRANS = Mat(st,st,CV_64F,Scalar::all(0));
    for(int i =0; i<st;i++)
    { 
         TRANS.at<float>(i,i) = 0.5;
         TRANS.at<float>(i,i+1) = 0.5;
         if(i == st-1)
             TRANS.at<float>(i,i) = 1;
    }


    EMIS = Mat(st, obs, CV_64F, Scalar::all(1/obs));
    
    double init[10] = {0.0};
    init[0] = 1.0;
    INIT = Mat(1,st,CV_64F,init);
  
    hmm.train(train_seq,100,TRANS,EMIS,INIT);
}


double decode(Mat seq, Mat TRANS, Mat EMIS, Mat INIT)
{
    Mat pstates,forward,backward;
    double logpseq;
    for (int i=0;i<seq.rows;i++)
    {
        hmm.decode(seq.row(i),TRANS,EMIS,INIT,logpseq,pstates,forward,backward);
    }
    return logpseq;
}


void hmm_train()
{
    int r = 6,c = 8;
    int obs = 4;
   
    //LEFT
    int st1 = 5;
    int data1[][8] = {{0,0,0},{0,0,0,0},{0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
    Mat seq1(r, c, CV_32S, data1);
    train(seq1, TRANS1, EMIS1, INIT1, st1, obs); 
    //hmm.printModel(TRANS1, EMIS1, INIT1);

    //RIGHT
    int st2 = 6;
    int data2[][8] = {{2,2,2},{2,2,2,2},{2,2,2,2,2},{2,2,2,2,2,2},{2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
    Mat seq2(r, c, CV_32S, data2);
    train(seq2, TRANS2, EMIS2, INIT2, st2, obs); 
    //hmm.printModel(TRANS2, EMIS2, INIT2);


    //UP
    int st3 = 4;
    int data3[][8] = {{3,3,3},{3,3,3,3},{3,3,3,3,3},{3,3,3,3,3,3},{3,3,3,3,3,3,3},{3,3,3,3,3,3,3,3}};
    Mat seq3(r, c, CV_32S, data3);
    train(seq3, TRANS3, EMIS3, INIT3, st3, obs); 
    //hmm.printModel(TRANS3, EMIS3, INIT3);


    //DOWN
    int st4 = 3;
    int data4[][8] = {{1,1,1},{1,1,1,1},{1,1,1,1,1},{1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1}};
    Mat seq4(r, c, CV_32S, data4);
    train(seq4, TRANS4, EMIS4, INIT4, st4, obs); 
    //hmm.printModel(TRANS4, EMIS4, INIT4);
}


void hmm_test()
{
    double log[4], max;
    int r,c,n,j;
    r = 1;
    n = 4;
    c = ang.size();

    Mat seq(r, c, CV_32S);
    seq = Mat(ang).reshape(0,1);

    log[0] = decode(seq, TRANS1, EMIS1, INIT1);
    log[1] = decode(seq, TRANS2, EMIS2, INIT2);
    log[2] = decode(seq, TRANS3, EMIS3, INIT3);
    log[3] = decode(seq, TRANS4, EMIS4, INIT4);

    max = -100;
    for(int i=0;i<4;i++)
    {
       if(log[i] > max)
       {
          max = log[i];
          n = i;
       }
    } 
  int p1 = 0;
  int p2 = 50;
  if(n<4)
  {
     switch(n)
     {
       case 0: { sprintf(t,"SWIPE LEFT"); break;}
       case 1: { sprintf(t,"SWIPE RIGHT"); break;}
       case 2: { sprintf(t,"SWIPE UP"); break;}
       case 3: { sprintf(t,"SWIPE DOWN"); break;}
     }
  }
  putText(im1, t, Point(p1,p2), FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255,255,255), 5.0);
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
 

int check()
{
   int flag = 0;
   int pix = countNonZero(im1);
   if(pix < 8000)
      flag = 1;
   return flag;
}


void calc()
{
   int c = check();
   if(c == 1)
   {
     Rect rect1, rect2;
     Mat im = im1.clone();
     Moments m = moments(im, true);
     Point cen(m.m10/m.m00, m.m01/m.m00);
     x.push_back(cen.x); 
     y.push_back(cen.y);
     z = x.size();
     if(z>1)
     {
        double at = atan2(y[z-1] - y[z-2], x[z-1] - x[z-2])*(180/CV_PI);
        if(at < 0) at = 360 + at;
        ang.push_back(vectorise(at));
     }
   }
   else
   {
       if(ang.size()>4)
       {   
           ang.erase(ang.begin());
           ang.erase(ang.begin());
           ang.pop_back();
           ang.pop_back(); 
           for(int i=0;i<ang.size();i++)
              cout<<ang[i]<<" ";
           cout<<endl; 
           hmm_test();
       }
       x.clear();
       y.clear();
       ang.clear();
   }
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
     
     //imshow(WR, img);
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
   
   int p = countNonZero(im1);

   if(p <100)
   {
        x.clear();
        y.clear();
        ang.clear();
   }
   if(q == 0)
      hmm_train();     

    if(min < 1.5)
    {
     calc();
    }   

    cv::imshow(WD, depth_frame);
    cv::imshow("GESTURE", im1);
    
    if(cv::waitKey(30)==27)
              exit(0);
    q = 1;
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
