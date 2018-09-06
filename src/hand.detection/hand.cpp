#include <stdio.h>
#include <iostream>
#include "opencv2/video/tracking.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/background_segm.hpp"
#include <opencv/cv.h>
#include "CvHMM.h"
#include "avr.h";

using namespace std;
using namespace cv;


BackgroundSubtractorMOG2 bg_model(100000, 5, false);
Mat im1, img,fgimg,fgmask,out, draw;
vector<std::vector<cv::Point> > contours;
vector<std::vector<cv::Point> > cont;
double area, pa;
Point2f cen;
float rad;
Scalar sc1 = Scalar(80,85,135);
Scalar sc2 = Scalar(240,135,180);
//Scalar sc1 = Scalar(0,85,135);
//Scalar sc2 = Scalar(240,135,180);
vector<Point> poly;
Point prev;
int dx, dy;
vector<Rect> faces;
CvHMM hmm;
vector<double> x, y;
vector<int> ang;
int z = 0;
char t[20];
Mat TRANS1, EMIS1, INIT1;
Mat TRANS2, EMIS2, INIT2;
Mat TRANS3, EMIS3, INIT3;
Mat TRANS4, EMIS4, INIT4;
string fac = "/home/tushardobhal/Desktop/OpenCV 245/cpp/xml/haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
int q = 0;

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
       case 0: { send("a");sprintf(t,"SWIPE LEFT"); break;}
       case 1: { send("d");sprintf(t,"SWIPE RIGHT"); break;}
       case 2: { send("w");sprintf(t,"SWIPE UP"); break;}
       case 3: { send("s");sprintf(t,"SWIPE DOWN"); break;}
     }
  }
  im1 = Mat(img.size(), img.type(), Scalar::all(0));
  putText(im1, t, Point(p1,p2), FONT_HERSHEY_PLAIN, 3.0, CV_RGB(255,255,255), 5.0);
  imshow("No.",im1);
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

void calc(int c, Point cen)
{
   if(c == 1)
   {
     Rect rect1, rect2;
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
        
        cv::erode(fgmask,fgmask,cv::Mat());
        
        cv::dilate(fgmask,fgmask,cv::Mat());
        fgimg = Scalar::all(0);
        img.copyTo(fgimg, fgmask);
        face_cascade.detectMultiScale( img, faces, 1.1, 4, CV_HAAR_FIND_BIGGEST_OBJECT|CV_HAAR_SCALE_IMAGE, Size(100, 100));
        if(faces.size() >= 1)
        fgimg(Range(faces[0].y+0.0*faces[0].height,faces[0].y+1.0*faces[0].height),Range(faces[0].x+0.0*faces[0].width,faces[0].x+1.0*faces[0].width)) = Scalar::all(0);
        cvtColor(fgimg,fgimg,CV_BGR2YUV);
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
       
       if(max > 1000)
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
           calc(1, hull[0][y]);
       }    
       else
           calc(0, Point(0,0)); 
}

int main()
{
     VideoCapture cap(0);
     if(!cap.isOpened())
        {
           cout<<"OOPS!!!";
           exit(0);
        }
     if(z == 0)   
     {
          hmm_train();
     }
     z = 1;
     //im1 = Mat(img.size(), img.type(), Scalar::all(0));
     while(1)
     {
        cap>>img;
        //im1 = Mat(img.size(), img.type(), Scalar::all(0));
        color();
         
        cvtColor(fgimg, fgimg, CV_YUV2BGR);
             
        char ch[20];
        sprintf(ch,"im/%d.jpg",z); 
        imshow("IMAGE", img);
        imshow("IMAGE1", fgimg);
        imwrite(ch,img);
        if(cv::waitKey(30)==27)
              exit(0);
         ++z;
      }
      return(0);
}


