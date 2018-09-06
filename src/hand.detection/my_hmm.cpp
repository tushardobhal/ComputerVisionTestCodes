#include <iostream>
#include <time.h>
#include <type_traits>
#include "opencv2/core/core.hpp"
#include <opencv2/core/core.hpp>
#include "CvHMM.h"

using namespace std;
using namespace cv;

CvHMM hmm;

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


double decode(Mat seq, Mat TRANS, Mat EMIS, Mat INIT, string d)
{
    cout <<"\n"<<d<<"\n";
    Mat pstates,forward,backward;
    double logpseq;
    for (int i=0;i<seq.rows;i++)
    {
        hmm.decode(seq.row(i),TRANS,EMIS,INIT,logpseq,pstates,forward,backward);
        cout << "logpseq" << i << " " << logpseq << "\n";
    }
    return logpseq;
}



int main()
{
    int r,c;
    int obs = 4;
  
    //LEFT
    int st1 = 5;
    int data1[][8] = {{0,0,0},{0,0,0,0},{0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
    r = std::extent<decltype(data1),0>::value;
    c = std::extent<decltype(data1),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    
    Mat TRANS1, EMIS1, INIT1;
    Mat seq1(r, c, CV_32S, data1);
    train(seq1, TRANS1, EMIS1, INIT1, st1, obs); 
    //hmm.printModel(TRANS1, EMIS1, INIT1);

    //RIGHT
    int st2 = 6;
    int data2[][8] = {{2,2,2},{2,2,2,2},{2,2,2,2,2},{2,2,2,2,2,2},{2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
    r = std::extent<decltype(data2),0>::value;
    c = std::extent<decltype(data2),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    
    Mat TRANS2, EMIS2, INIT2;
    Mat seq2(r, c, CV_32S, data2);
    train(seq2, TRANS2, EMIS2, INIT2, st2, obs); 
    //hmm.printModel(TRANS2, EMIS2, INIT2);


    //UP
    int st3 = 4;
    int data3[][8] = {{3,3,3},{3,3,3,3},{3,3,3,3,3},{3,3,3,3,3,3},{3,3,3,3,3,3,3},{3,3,3,3,3,3,3,3}};
    r = std::extent<decltype(data3),0>::value;
    c = std::extent<decltype(data3),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    
    Mat TRANS3, EMIS3, INIT3;
    Mat seq3(r, c, CV_32S, data3);
    train(seq3, TRANS3, EMIS3, INIT3, st3, obs); 
    //hmm.printModel(TRANS3, EMIS3, INIT3);


    //DOWN
    int st4 = 3;
    int data4[][8] = {{1,1,1},{1,1,1,1},{1,1,1,1,1},{1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1}};
    r = std::extent<decltype(data4),0>::value;
    c = std::extent<decltype(data4),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    
    Mat TRANS4, EMIS4, INIT4;
    Mat seq4(r, c, CV_32S, data4);
    train(seq4, TRANS4, EMIS4, INIT4, st4, obs); 
    //hmm.printModel(TRANS4, EMIS4, INIT4);

 
    //test
    int data[] = {3,0,0,0,0,0,2};
    double log[4], max;
    char t[20];
    int n = 0;
    r = std::extent<decltype(data),0>::value;
    c = std::extent<decltype(data),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    Mat seq(r, c, CV_32S, data);
    log[0] = decode(seq, TRANS1, EMIS1, INIT1, "LEFT");
    log[1] = decode(seq, TRANS2, EMIS2, INIT2, "RIGHT");
    log[2] = decode(seq, TRANS3, EMIS3, INIT3, "UP");
    log[3] = decode(seq, TRANS4, EMIS4, INIT4, "DOWN");

    max = log[0];
    for(int i=0;i<4;i++)
    {
       if(log[i] > max)
       {
          max = log[i];
          n = i;
       }
    }
  switch(n)
  {
      case 0: { sprintf(t,"SWIPE LEFT"); break;}
      case 1: { sprintf(t,"SWIPE RIGHT"); break;}
      case 2: { sprintf(t,"SWIPE UP"); break;}
      case 3: { sprintf(t,"SWIPE DOWN"); break;}
  }
  cout<<t<<endl;
}

//g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename my_hmm.cpp .cpp` my_hmm.cpp `pkg-config --libs opencv`



/*

1. LEFT   -  0
2. RIGHT  -  2
3. UP     -  3
4. DOWN   -  1

*/
