#include<iostream>
#include<math.h>
#include<cmath>

double getDistance(double lat1, double lon1, double lat2, double lon2)
{
    double earthRadius = 6371000;
    double dLat = (lat2-lat1)*3.14/180.0;
    double dLon = (lon2-lon1)*3.14/180.0;
    
    lat1 = lat1*3.14/180.0;
    lat2 = lat2*3.14/180.0;
    lon1 = lon1*3.14/180.0;

    double a = sin(dLat/2) * sin(dLat/2) +
          sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2); 
  double c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  return earthRadius * c;
}

int main()
{
    std::cout<<getDistance(28.5487534, 77.2740308, 28.5487435, 77.2740309)<<std::endl;
    return 0;
}