#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;



void detect(Mat &im)
{
    Mat img; 
    cvtColor( im, img, CV_BGR2GRAY );
    equalizeHist( img, img );
   
    string fac, eye, sm;
    CascadeClassifier face_cascade, eye_cascade, smile_cascade;
    fac = "xml/haarcascade_frontalface_alt.xml";
    //eye = "xml/haarcascade_eye_tree_eyeglasses.xml";
    //sm = "xml/haarcascade_smile.xml";

    if(!face_cascade.load(fac)){cout<<"ERROR Loading XML Files";}
    //if(!eye_cascade.load(eye)){cout<<"ERROR Loading XML Files";}
    //if(!smile_cascade.load(sm)){cout<<"ERROR Loading XML Files";}   
    
    

    vector<Rect> faces;
    face_cascade.detectMultiScale( img, faces, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));

    int i,j;
    for(i=0;i<faces.size();i++)
    {
         //Rect rect(faces[i].x,faces[i].y,faces[i].width,faces[i].height);
         //rectangle(im,rect,Scalar(0,0,255));
         Point center(faces[i].x+faces[i].width/2, faces[i].y+faces[i].height/2);
         Size sz(faces[i].width/2,faces[i].height/2);
         ellipse(im,center,sz,0,0,360,Scalar(0,0,255),2,8,0);

         Mat roi = img(faces[i]);

         /*
		 vector<Rect> smile;
         smile_cascade.detectMultiScale(roi,smile, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));
         for(j=0;j<smile.size();j++)
         {
              Point center(faces[i].x+smile[j].x+smile[j].width/2, faces[i].y+smile[j].y+smile[j].height/2);
              Size sz(smile[j].width/2,smile[j].height/2);
              ellipse(im,center,sz,0,0,360,Scalar(0,0,255),2,8,0);
            
         }
		 */

         /* vector<Rect> eyes; 
         eye_cascade.detectMultiScale(roi,eyes, 1.1, 3, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));
         for(j=0;j<eyes.size();j++)
         {
              Point center(faces[i].x+eyes[j].x+eyes[j].width/2, faces[i].y+eyes[j].y+eyes[j].height/2);
              Size sz(eyes[j].width/2,eyes[j].height/2);
              ellipse(im,center,sz,0,0,360,Scalar(0,0,255),2,8,0);
         } */
		 
    }

   imshow("Image1",im);
}

int main()
{
     VideoCapture cap(0);
	 Mat im;
     if(!cap.isOpened())
     {
         cout<<"OOPS!!!";
         return(-1);
     }

	 while(1)
	 {
        
		cap>>im;
	    detect(im);
		
		
		if(waitKey(1) == 27)
			break;
	 }
	waitKey();
    return(0);
}
