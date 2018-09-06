#include <opencv\cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <conio.h>

using namespace cv;
using namespace std;

int ma()
{
    Mat im2 = imread("C:/Projects/opencv/72ab24a2-227b-4ad7-b83c-d6a49595486c (2).jpg",CV_LOAD_IMAGE_COLOR);
	Mat im = imread("C:/Users/Tushar Dobhal/Pictures/From Tushar/Saved pictures/eric-cartman_55528.jpg",CV_LOAD_IMAGE_COLOR);
    if (im2.empty()) 
    {
        cout << "Cannot load image!" << endl;
	    waitKey();
        return -1;
    }
	Mat roi = im(Rect((im.cols/2)-(im2.cols/2),(im.rows/2)-(im2.rows/2),im2.cols,im2.rows));
	addWeighted(roi,0,im2,1,0,roi);
	
	namedWindow("Image",CV_WINDOW_AUTOSIZE);
	imshow("Image",im);
	
	waitKey();
	return(0);
}