#include <opencv\cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <conio.h>

using namespace cv;
using namespace std;

int mai()
{
    Mat im = imread("C:/Projects/opencv/72ab24a2-227b-4ad7-b83c-d6a49595486c (2).jpg",CV_LOAD_IMAGE_GRAYSCALE);
	
    if (im.empty()) 
    {
        cout << "Cannot load image!" << endl;
	    waitKey();
        return -1;
    }
	int i,j;
	int r = im.rows;
	int c = im.cols;

	/*Mat im3; 
	im3.create(200,200,CV_8UC3);             To Create a Blank Matrix
	
	Mat im4(200,200,CV_8UC3,Scalar(255,0,0));   To Create a Matrix with certain Color 
	*/


	Mat im1 = im.clone();
	Mat im2;
	im2.create(im.rows,im.cols,im.type());
	for(i=1;i<r-1;i++)
		for(j=1;j<c-1;j++)
		{
			im2.at<uchar>(i,j) = saturate_cast<uchar>((5*im.at<uchar>(i,j))-(im.at<uchar>(i-1,j)+im.at<uchar>(i,j-1)+im.at<uchar>(i+1,j)+im.at<uchar>(i,j+1)));
		}

		im2.row(0).setTo(Scalar(0));
		im2.row(r-1).setTo(Scalar(0));
		im2.col(0).setTo(Scalar(0));
		im2.col(c-1).setTo(Scalar(0));

cv::Mat kernel(3,3,CV_32F,Scalar(0));
kernel.at<float>(1,1)= 5.0;
kernel.at<float>(0,1)= -1.0;
kernel.at<float>(2,1)= -1.0;
kernel.at<float>(1,0)= -1.0;
kernel.at<float>(1,2)= -1.0;
cv::filter2D(im,im1,im.depth(),kernel);
	
		
	namedWindow("Image",CV_WINDOW_AUTOSIZE);
	namedWindow("Image1",CV_WINDOW_AUTOSIZE);
	namedWindow("Image2",CV_WINDOW_AUTOSIZE);
	imshow("Image",im);
	imshow("Image1",im1);
	imshow("Image2",im2);
	waitKey();
	return(0);
}