#include <vector>
#include <utility>
#include <cmath>

using namespace std;

class DTW 
{
   double euclid(vector<double> p1, vector<double> p2)
    {
         double sum = 0, res;
         for(int i=0;i<p1.size();i++)
         {
              sum += pow((p1[i] - p2[i]),2); 
         }
         res = sqrt(sum);
         return res;
    }
   

   double dist(double x, double y) 
    {
        return sqrt(pow((x - y), 2));
    }
   
 public:

   double compDTW(const std::vector<double>& t1, const std::vector<double>& t2) 
   {
        int m = t1.size();
        int n = t2.size();

        // create cost matrix
        double cost[m][n];

        cost[0][0] = dist(t1[0], t2[0]);

        // calculate first row
        for(int i = 1; i < m; i++)
            cost[i][0] = cost[i-1][0] + dist(t1[i], t2[0]);
        // calculate first column
        for(int j = 1; j < n; j++)
            cost[0][j] = cost[0][j-1] + dist(t1[0], t2[j]);
        // fill matrix
        for(int i = 1; i < m; i++)
            for(int j = 1; j < n; j++)
                cost[i][j] = std::min(cost[i-1][j], std::min(cost[i][j-1], cost[i-1][j-1])) 
                    + dist(t1[i],t2[j]);

        return cost[m-1][n-1];
    }


    double compDTW(vector<vector<double> > t1, vector<vector<double> > t2)
    {
        int m = t1.size();
        int n = t2.size();
        
        double dis[m][n];
        for(int i=0;i<m;i++)
        {
           for(int j=0;j<n;j++)
           {
               dis[i][j] = euclid(t1[i], t2[j]);
           }
        }
        
        double cost[m][n];

        cost[0][0] = dis[0][0];

        for(int i = 1; i < m; i++)
            cost[i][0] = cost[i-1][0] + dis[i][0];
        
        for(int j = 1; j < n; j++)
            cost[0][j] = cost[0][j-1] + dis[0][j];
        
        for(int i = 1; i < m; i++)
            for(int j = 1; j < n; j++)
                cost[i][j] = min(cost[i-1][j], min(cost[i][j-1], cost[i-1][j-1])) + dis[i][j];

        return cost[m-1][n-1];
    }  
};
