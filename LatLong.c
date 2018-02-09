//Created by Rohan J. Dani
#include <stdio.h>
#include <math.h>
#include <vector>
int LatLong(std::vector<int> location) {
  int ECEFx;
  int ECEFy;
  int ECEFz; //get ECEF location to be converted to latitude-longitude-altitude coords
  double value;
  double p;
  double lat0;
  int stop = 0;
  int ECEFx = location[1];
  int ECEFy = location[2];
  int ECEFz = location[3];
  value = atan2(ECEFy,ECEFx); //compute the longitude which is an exact calculation
  p = sqrt(ECEFx^2 + ECEFy^2); //compute the latitude using iteration
  lat0 = atan2(ECEFz,p); //compute approximate latitude
  while(stop == 0) {
    //N0 = AA^2/sqrt((AA^2*(cos(lat0)^2))+(BB^2*sin(lat0)^2)) ; what is AA, BB
    //altitude = (p/(cos(lat0)))-N0 ;
  }
}
