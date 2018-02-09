//Created by Rohan J. Dani
//Converts Julian Date to unit vector direction to SUn in ECIF
#include <math.h>
#include <stdio.h>
#define sind(x) (sin(fmod((x),360) * M_PI / 180))
#define cosd(x) (cos(fmod((x),360) * M_PI / 180))
#define pi 3.14159265358979323846
void JD2SunInertial(double JD) {
  double Ttdb;
  double Tut1;
  double lambdaMSun;
  double Msun;
  double lambdaecliptic;
  double epsilon;
  double si;
  double sir;
  double sin2Msun;
  double sin2Msun;
  double error;
  double variance;
  double sinepsilon;
  double sinlambdaepsilon;
  double coslambdaepsilon;
  double cosepsilon;
  Tut1=(JD - 2451545.0)/36525;
  lambdaMSun=280.4606184+36000.77005361*Tut1;
  Ttdb=Tut1;
  Msun=357.5277233+35999.05034*Ttdb;
  lambdaecliptic=lambdaMSun+1.914666471*sind(Msun)+0.918994643*sind(2*Msun);
  epsilon=23.439291-0.0130042*Ttdb;
  si=[cosd(lambdaecliptic), cosd(epsilon)*sind(lambdaecliptic),sind(epsilon)*sind(lambdaecliptic)];
  lambdaMSun=(280.4606184+36000.77005361*Tut1)*(pi/180);
  Ttdb=Tut1;
  Msun=(pi/180)*(357.5277233+35999.05034*Ttdb);
  lambdaecliptic=lambdaMSun+(pi/180)*(1.914666471)*sin(Msun)+(pi/180)*0.918994643*sin(2*Msun);
  epsilon=(pi/180)*(23.439291-0.0130042*Ttdb);
  sir=[cos(lambdaecliptic), cos(epsilon)*sin(lambdaecliptic),sin(epsilon)*sin(lambdaecliptic)];

  //Taylor Series
  double Msun1;
  Msun1 = reduceRadians(Msun);
  sinMsun = Taylor(Msin1,6);
  sin2Msun = Taylor(2*Msun1,6);

  lambdaecliptic=lambdaMSun+(pi/180)*(1.914666471)*sinMsun+(pi/180)*0.918994643*sin2Msun;
  epsilon=(pi/180)*(23.439291-0.0130042*Ttdb);
  lambdaecliptic=reduceRadians(lambdaecliptic);
  sinepsilon=Taylor(epsilon,6);
  cosepsilon=Taylor(epsilon,6);
  sinlambdaepsilon=Taylor(lambdaecliptic,6);
  coslambdaepsilon=Taylor(lambdaecliptic,6);
  epsilon=reduceRadians(epsilon);

  sirTaylor=[coslambdaecliptic, cosepsilon*sinlambdaecliptic,sinepsilon*sinlambdaecliptic];

  error=sirTaylor-sir;
  variance = sqrt(error*error);
}
