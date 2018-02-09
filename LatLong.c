//Created by Rohan J. Dani
#include <stdio.h>
#include <math.h>
#include <vector>
#ifndef CONSTANT_C
#define CONSTANT_C
int LatLong(std::vector<int> location) {
  extern
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
    N0 = AA^2/sqrt((AA^2*(cos(lat0)^2))+(BB^2*sin(lat0)^2));
    altitude = (p/(cos(lat0)))-N0 ;
    term = p*(1 - esquare*(N0/(N0 + altitude)));
    lat  = atan(ECEFz/(p*(1-(esquare*N0/(N0+altitude)))));
    if (abs(lat - lat0) == 0) {
      stop = 1;
    }
    lat0 = lat;
  }
  latitude = lat / degrad;		% degrees
	longitude = value / degrad;
  std::vector<int> = {latitude,longitude,altitude};
}
#endif
#include "constant.c"
