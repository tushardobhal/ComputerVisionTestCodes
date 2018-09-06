#include <stdio.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"
#include <opencv/cv.h>

using namespace std;
using namespace cv;


BackgroundSubtractorMOG2 bg_model(100000, 5, false);
Mat im1, img,fgimg,fgmask,out, draw;
vector<std::vector<cv::Point> > contours;
vector<std::vector<cv::Point> > cont;
double area, pa;
Point2f cen;
float rad;
Scalar sc1 = Scalar(80,135,85);
Scalar sc2 = Scalar(240,180,135);
//Scalar sc1 = Scalar(0,85,135);
//Scalar sc2 = Scalar(240,135,180);
vector<Point> poly;
Point prev;
int dx, dy;
vector<Rect> faces;
string fac = "../xml/haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
int q = 0;



void color()
{
    int flag = 0;
    if(q == 0)
        if(!face_cascade.load(fac)) cout<<"OOPS!!! FILE";
    q = 1;
    if( fgimg.empty() )
       fgimg.create(img.size(), img.type());
       
        //update the model
        bg_model(img, fgmask, -1);
        
        //cv::erode(fgmask,fgmask,cv::Mat());
        
        //cv::dilate(fgmask,fgmask,cv::Mat());
        fgimg = Scalar::all(0);
        img.copyTo(fgimg, fgmask);
        face_cascade.detectMultiScale( img, faces, 1.1, 4, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(100, 100));
        fgimg(Range(faces[0].y+0.0*faces[0].height,faces[0].y+1.0*faces[0].height),Range(faces[0].x+0.0*faces[0].width,faces[0].x+1.0*faces[0].width)) = Scalar::all(0);
        cvtColor(fgimg,fgimg,CV_BGR2YCrCb);
        inRange(fgimg, sc1, sc2,out);
        cv::erode(out,out,cv::Mat());
        
        cv::dilate(out,out,cv::Mat());
       double max = 0;
       int x =0;
       findContours(out,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
       vector<vector<Point> > hull(1);
       vector<vector<int> > hulli(1); 
       vector<Vec4i> def;
       for(int i=0;i<contours.size();i++)
       {
            area = contourArea(contours[i]);
            if(area>max)
                {
                    max = area;
                    x=i;
                }
       }
       cout<<max<<'\n';
       if(max > 10000)
       {
           int miny = 480;
           int y =0;
           convexHull(Mat(contours[x]), hull[0], false);
          
           approxPolyDP(Mat(hull[0]), hull[0], 0.001, true);
           drawContours(img, hull, 0, Scalar(0,0,255), 2);
           for(int j=0;j<hull[0].size();j++)
           {
                   if(hull[0][j].y < miny)
                       {
                          miny = hull[0][j].y;
                          y = j;
                        }
           }
          
           circle(img, hull[0][y], 5, Scalar(0,255,0), -1); 
       }    
}

int main()
{
     VideoCapture cap(0);
     if(!cap.isOpened())
        {
           cout<<"OOPS!!!";
           exit(0);
        }
     int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
     int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
     VideoWriter videoFinal("namo.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height),true);

     while(1)
     {
        cap>>img;
        //im1 = Mat(img.size(), img.type(), Scalar::all(0));
        color();
         
        cvtColor(fgimg, fgimg, CV_YCrCb2BGR);
             
         
        imshow("IMAGE", img);
        imshow("IMAGE1", fgimg);
	videoFinal.write(img);
        if(cv::waitKey(30)==27)
              exit(0);
      }
      return(0);
}


