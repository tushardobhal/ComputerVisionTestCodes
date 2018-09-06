#include "include/MFCC.h"
#include "include/MIC.h"
#include "include/DTW.h"
#include "include/kbhit.h"
#include "include/avr.h"
#include <string.h>

using namespace std;

MFCC mfcc;
DTW dtw;
MIC mic;

int tot = 6;


vector<char> tovec(char*s)
{
    vector<char> r;
    for(int i=0;i<strlen(s);i++)
        r.push_back(s[i]);
    return r;
}


int num(int f)
{
    for(int i=0;i<tot;i++)
    {
         if((f - 4*i) < 4)
            return (4*i);
    }
}


int main()
{
  int z = 0;
  char l[50],r[50],f[50],s[50];
  char in_t[] = "output.wav";
  vector<vector<char> > name;
  for(int i=0;i<tot;i++)
  {
      sprintf(l, "files/wav/left%d", i);
      name.push_back(tovec(l));
      sprintf(r, "files/wav/right%d", i);
      name.push_back(tovec(r));
      sprintf(f, "files/wav/forward%d", i);
      name.push_back(tovec(f));
      sprintf(s, "files/wav/stop%d", i);
      name.push_back(tovec(s));
  }
  vector<vector<vector<double> > > mfcc1;
  vector<vector<double> > mfcc_t;
  vector<double> cost;
  char c;
  int n;
  while(1)
  {
     if(z == 0)
     {
          for(int i=0;i<tot;i++)
          {
              mfcc1.push_back(mfcc.compMFCC(name[i*4].data(), name[i*4].size()));
              mfcc1.push_back(mfcc.compMFCC(name[i*4+1].data(), name[i*4+1].size()));
              mfcc1.push_back(mfcc.compMFCC(name[i*4+2].data(), name[i*4+2].size()));
              mfcc1.push_back(mfcc.compMFCC(name[i*4+3].data(), name[i*4+3].size()));
          }
     }
     else
     {
          printf("\r%s","PRESS SPACEBAR for MIC Input, ESC to QUIT");
          fflush(stdout);
          if(kbhit())
          {
               c = getchar();
               if(c == 32)
               {
                   mic.record();
                   mfcc_t = mfcc.compMFCC(in_t, 0);    
                   for(int i=0;i<tot;i++)
                   {
                       cost.push_back(dtw.compDTW(mfcc_t,mfcc1[i*4]));
                       cost.push_back(dtw.compDTW(mfcc_t,mfcc1[i*4+1]));
                       cost.push_back(dtw.compDTW(mfcc_t,mfcc1[i*4+2]));
                       cost.push_back(dtw.compDTW(mfcc_t,mfcc1[i*4+3]));
                   }
                   double min = cost[0];
                   n = 0;
                   for(int i=0;i<cost.size();i++)
                   {
                      cout<<cost[i]<<" ";
                      if(cost[i] < min)
                      {
                          min = cost[i];
                          n = i;
                      }
                   }
                   system("clear");
                   switch(n - num(n))
                   {
                       cout<<"\n"<<n<<"\n";
                       case 0:{send("a");cout<<"LEFT\n";break;}
                       case 1:{send("d");cout<<"RIGHT\n";break;}     
                       case 2:{send("w");cout<<"FORWARD\n";break;}
                       case 3:{send("s");cout<<"STOP\n";break;}  
                   }
                   cost.clear();
               }
               if(c == 27)
               {
                   system("clear"); 
                   exit(0); 
               }
          }            
      }
     z = 1;
  } 
  
   return 1;
}   

//g++ -main.cpp -lsndfile -lfftw3 -lportaudio 
