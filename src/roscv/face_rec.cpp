#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include "facerec.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

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
  
cv::Mat rgb_frame;
cv::Mat depth_frame;
cv::Mat image,face,preprocessed,face_resized;
string fn_csv,name;
vector<Mat> images;
vector<int> labels;
int im_height,im_width;
LBPH model;
vector<Rect> faces;
CascadeClassifier face_cascade;
string fac;
int i;
int prediction;
double confidence;


ImageConverter()
    : it_(nh_)
  {
    
    image_pub_ = it_.advertise("out", 1);
    depth_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth_registered/image", 1, &ImageConverter::depthInfoCb, this);

   
   
    fac = "/u/pr2admin/facerec/haarcascade_frontalface_alt_tree.xml";
    
    if(!face_cascade.load(fac)){cout<<"ERROR Loading XML Files";}
    

    fn_csv = "/u/pr2admin/facerec/im.txt";
 
    
    try {
        read_csv(fn_csv, images, labels);
    } catch (cv::Exception& e) {
        cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
       
        exit(1);
    }
   
    im_width = images[0].cols;
    im_height = images[0].rows;

   
    model.train(images, labels);
    

    
       
      cv::namedWindow(WR,CV_WINDOW_AUTOSIZE);
      cv::namedWindow(WD, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WR);
    cv::destroyWindow(WD);
  }


static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}



Mat norm_0_255(const Mat& src) {
    
    Mat dst;
    switch(src.channels()) {
    case 1:
        cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
        break;
    case 3:
        cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC3);
        break;
    default:
        src.copyTo(dst);
        break;
    }
    return dst;
}


Mat tan_triggs_preprocessing(InputArray src,
        float alpha = 0.1, float tau = 10.0, float gamma = 0.2, int sigma0 = 1,
        int sigma1 = 2) {

  
    Mat X = src.getMat();
    X.convertTo(X, CV_32FC1);
    
    Mat I;
    pow(X, gamma, I);

    {
        Mat gaussian0, gaussian1;
    
        int kernel_sz0 = (3*sigma0);
        int kernel_sz1 = (3*sigma1);
        
        kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
        kernel_sz1 += ((kernel_sz1 % 2) == 0) ? 1 : 0;
        GaussianBlur(I, gaussian0, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_CONSTANT);
        GaussianBlur(I, gaussian1, Size(kernel_sz1,kernel_sz1), sigma1, sigma1, BORDER_CONSTANT);
        subtract(gaussian0, gaussian1, I);
    }

    {
        double meanI = 0.0;
        {
            Mat tmp;
            pow(abs(I), alpha, tmp);
            meanI = mean(tmp).val[0];

        }
        I = I / pow(meanI, 1.0/alpha);
    }

    {
        double meanI = 0.0;
        {
            Mat tmp;
            pow(min(abs(I), tau), alpha, tmp);
            meanI = mean(tmp).val[0];
        }
        I = I / pow(meanI, 1.0/alpha);
    }

    {
        for(int r = 0; r < I.rows; r++) {
            for(int c = 0; c < I.cols; c++) {
                I.at<float>(r,c) = tanh(I.at<float>(r,c) / tau);
            }
        }
        I = tau * I;
    }
    return I;
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
    
      rgb_frame = cv_ptr_rgb->image;
       
      cvtColor(rgb_frame, image, CV_BGR2GRAY );
      equalizeHist( image, image );
       
     face_cascade.detectMultiScale(image, faces, 1.1, 4, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));
     
    
     for(i=0;i<faces.size();i++)
    {
            prediction =-1;
           confidence = 0; 
          Rect rect(faces[i].x + 0.1*faces[i].width,faces[i].y+0.1*faces[i].height,0.8*faces[i].width,0.9*faces[i].height);

           face = rgb_frame(rect);
           rectangle(rgb_frame,rect,Scalar(0,255,0), 2.0);
           
           cvtColor(face,face,CV_BGR2GRAY);
           equalizeHist(face,face);
        
           preprocessed = tan_triggs_preprocessing(face);  
           face = norm_0_255(preprocessed);

           cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
            
                

           
            model.predict(face_resized,prediction,confidence);
            cout<<"pred "<<prediction<<"\t";
            cout<<"confidence"<<confidence<<endl;
            
             
            if(prediction==0)
		{

                          string name = "Welcome Usha"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }
            if(prediction==1)
                {

                          string name = "Welcome Krishna"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }      
             if(prediction==2)
                {

                          string name = "Welcome Anil"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }      
              if(prediction==3)
                {

                          string name = "Welcome Megha"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }      
              if(prediction==4)
                {

                          string name = "Welcome Tushar"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }      
              if(prediction==5)
                {

                          string name = "Welcome Mani"; 
                          string command = "echo \"" + name  + "\" | festival --tts";
                          system(command.c_str());
                 }        
               /*  case 1: { name = "Mam\0"; break;}
                 case 2: { name = "Megha\0"; break;}
                 case 3: { name = "Tushar\0"; break;}
                 default:{ name = "seagtasrg\0";break;}*/ 
             
           
            
      
            string box_text = format("pred = %d Conf = %f", prediction, confidence);
            //cout<<box_text<<endl;
            
            
            putText(rgb_frame, box_text, Point(faces[i].x, faces[i].y-10), FONT_HERSHEY_PLAIN, 3.0, CV_RGB(0,255,0), 2.0);
            
       
        
    }


     cv::imshow(WR, rgb_frame);
     if(cv::waitKey(3)==27)
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
  ros::spin();
  return 0;
}


