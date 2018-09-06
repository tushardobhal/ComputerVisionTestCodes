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


double decode(Mat seq, Mat TRANS, Mat EMIS, Mat INIT, int d)
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
   
    //Digit - 0
    int st0 = 4;
    int data0[][12] = {{2,1,0,3},{2,2,1,1,0,0,3,3},{2,2,2,1,1,0,0,0,3,3},{2,2,1,1,0,0,0,3,3,3},{2,2,2,1,1,1,0,0,0,3,3,3},{2,2,1,1,1,1,0,0,0,3,3,3},{2,2,2,1,1,0,0,0,0,3,3,3},{2,2,2,1,1,1,0,0,3,3,3,3}};
    r = std::extent<decltype(data0),0>::value;
    c = std::extent<decltype(data0),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
   
    Mat TRANS0, EMIS0, INIT0;
    Mat seq0(r, c, CV_32S, data0);
    train(seq0, TRANS0, EMIS0, INIT0, st0, obs); 
    //hmm.printModel(TRANS0, EMIS0, INIT0);
  
    //Digit - 1
    int st1 = 1;
    int data1[][8] = {{1},{1,1},{1,1,1},{1,1,1,1},{1,1,1,1,1},{1,1,1,1,1,1},{1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1}};
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

    //Digit - 2
    int st2 = 5;
    int data2[][13] = {{2,1,0,1,2},{2,2,1,1,0,0,1,1,2,2},{2,2,2,1,1,0,0,0,1,1,2,2,2},{2,2,1,1,1,0,0,1,1,2,2,2},{2,1,1,0,0,0,1,1,1,2,2},{2,2,2,1,1,1,0,0,1,1,2,2},{2,2,1,1,1,0,0,0,1,1,2,2},{2,2,1,1,0,0,0,1,1,1,2,2},{2,2,1,1,0,0,1,1,1,2,2,2}};
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


    //Digit - 3
    int st3 = 6;
    int data3[][15] = {{2,1,0,2,1,0},{2,2,1,1,0,0,2,2,1,1,0,0},{2,2,2,1,1,0,0,0,2,2,2,1,1,0,0},{2,2,1,1,1,0,0,2,1,1,0,0,0},{2,2,1,1,0,0,2,2,1,1,0,0},{2,2,2,1,1,0,0,2,1,0,0,0},{2,2,1,1,0,0,0,2,2,2,1,0},{2,1,1,1,0,0,2,2,1,1,1,0}};
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


    //Digit - 4
    int st4 = 3;
    int data4[][12] = {{1,2,1},{1,1,2,2,1,1},{1,1,1,2,2,2,1,1,1},{1,1,2,2,2,1,1,1},{1,1,2,2,1,1,1},{1,1,2,2,2,1,1},{1,1,1,2,2,1,1,1},{1,1,1,2,2,2,2,1,1,1,1},{1,1,1,1,2,2,2,2,1,1,1,1},{1,1,1,2,2,2,2,2,1,1,1,1},{1,1,1,1,2,2,2,1,1,1,1,1},{1,1,1,1,2,2,2,2,2,1,1,1}};
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
    int data[] = {2,2,2,1,1,0,0,0,0,2,2,1,1,1,0,0,0,0};
    double log0,log1,log2,log3,log4;
    r = std::extent<decltype(data),0>::value;
    c = std::extent<decltype(data),1>::value;
    if(c == 0)
    {
       c = r;
       r = 1;
    }
    Mat seq(r, c, CV_32S, data);
    log0 = decode(seq, TRANS0, EMIS0, INIT0, 0);
    log1 = decode(seq, TRANS1, EMIS1, INIT1, 1);
    log2 = decode(seq, TRANS2, EMIS2, INIT2, 2);
    log3 = decode(seq, TRANS3, EMIS3, INIT3, 3);
    log4 = decode(seq, TRANS4, EMIS4, INIT4, 4);
}

//g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename my_hmm.cpp .cpp` my_hmm.cpp `pkg-config --libs opencv`



/*

0 - {{2,2,2,1,1,1,0,0,0,3,3,3},{2,2,1,1,1,1,0,0,0,3,3,3},{2,2,2,1,1,0,0,0,0,3,3,3},{2,2,2,1,1,1,0,0,3,3,3,3}};
1 - {{1,1,1,1,1,1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1,1,1,1,1},{1,1,1,1,1,1,1,1,1,1,1,1}};
2 - {{2,2,2,1,1,1,0,0,1,1,2,2},{2,2,1,1,1,0,0,0,1,1,2,2},{2,2,1,1,0,0,0,1,1,1,2,2},{2,2,1,1,0,0,1,1,1,2,2,2}};
3 - {{2,2,1,1,0,0,2,2,1,1,0,0},{2,2,2,1,1,0,0,2,1,0,0,0},{2,2,1,1,0,0,0,2,2,2,1,0},{2,1,1,1,0,0,2,2,1,1,1,0}};
4 - {{1,1,1,1,2,2,2,2,1,1,1,1},{1,1,1,2,2,2,2,2,1,1,1,1},{1,1,1,1,2,2,2,1,1,1,1,1},{1,1,1,1,2,2,2,2,2,1,1,1}};

*/
