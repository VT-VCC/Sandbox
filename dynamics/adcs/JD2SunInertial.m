function [si,sir,sirTaylor,variance]=JD2SunInertial(JD)
%Converts Julian Date to unit vector direction to SUn in ECIF
Tut1=(JD-2451545.0)/36525;
lambdaMSun=280.4606184+36000.77005361*Tut1;
Ttdb=Tut1;
Msun=357.5277233+35999.05034*Ttdb;
lambdaecliptic=lambdaMSun+1.914666471*sind(Msun)+0.918994643*sind(2*Msun);
epsilon=23.439291-0.0130042*Ttdb;
si=[cosd(lambdaecliptic), cosd(epsilon)*sind(lambdaecliptic),sind(epsilon)*sind(lambdaecliptic)]'; 

lambdaMSun=(280.4606184+36000.77005361*Tut1)*(pi/180);
Ttdb=Tut1;
Msun=(pi/180)*(357.5277233+35999.05034*Ttdb);




lambdaecliptic=lambdaMSun+(pi/180)*(1.914666471)*sin(Msun)+(pi/180)*0.918994643*sin(2*Msun);
epsilon=(pi/180)*(23.439291-0.0130042*Ttdb);
sir=[cos(lambdaecliptic), cos(epsilon)*sin(lambdaecliptic),sin(epsilon)*sin(lambdaecliptic)]'; 

%% Taylor Series 

[Msun1]=reduceRadians(Msun);


[sinMsun,~,~,~]=Taylor(Msun1,6);
[sin2Msun,~,~,~]=Taylor(2*Msun1,6);

lambdaecliptic=lambdaMSun+(pi/180)*(1.914666471)*sinMsun+(pi/180)*0.918994643*sin2Msun;
epsilon=(pi/180)*(23.439291-0.0130042*Ttdb);

[lambdaecliptic]=reduceRadians(lambdaecliptic);
[epsilon]=reduceRadians(epsilon);
[sinepsilon,cosepsilon,~,~]=Taylor(epsilon,6);
[sinlambdaecliptic,coslambdaecliptic,~,~]=Taylor(lambdaecliptic,6);
sirTaylor=[coslambdaecliptic, cosepsilon*sinlambdaecliptic,sinepsilon*sinlambdaecliptic]'; 

%% Error
error=sirTaylor-sir;
variance=sqrt(error'*error);

